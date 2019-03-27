/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

/*
 *  btstack_run_loop_ecos.c
 *
 *  For this run loop, we assume that there's no global way to wait for a list
 *  of data sources to get ready. Instead, each data source has to queried
 *  individually. Calling ds->isReady() before calling ds->process() doesn't 
 *  make sense, so we just poll each data source round robin.
 *
 *  To support an idle state, where an MCU could go to sleep, the process function
 *  has to return if it has to called again as soon as possible
 *
 *  After calling process() on every data source and evaluating the pending timers,
 *  the idle hook gets called if no data source did indicate that it needs to be
 *  called right away.
 *
 */

#include <cyg/kernel/kapi.h>
#include <cyg/hal/drv_api.h>

#include "btlib/btstack_run_loop.h"
#include "btlib/btstack_run_loop_ecos.h"
#include "btlib/btstack_linked_list.h"


#include "btlib/btstack_debug.h"

#include <stddef.h> // NULL

#define SYSTEM_TICK_TO_MS_CONST     10
// 10 means start run loop every 100 ms
#define SYSTEM_TIMER_COUNTS         10


// the run loop
static const btstack_run_loop_t btstack_run_loop_ecos;

// data sources
static btstack_linked_list_t data_sources = NULL;

// timers
static btstack_linked_list_t timers = NULL;

// structures to create timer counter
static cyg_alarm		timer_alarm;
static cyg_handle_t		timer_alarmH    = 0;
static cyg_handle_t		timer_counterH	= 0;
static cyg_handle_t		system_clockH	= 0;

static int trigger_event_received      = 0;

cyg_drv_mutex_t             transfer_mx;          // Transfer mutex
cyg_drv_cond_t              transfer_cond;        // Transfer condition

void btstack_ecos_timer_callback(cyg_handle_t alarmH, cyg_addrword_t data);
void btstack_run_loop_ecos_trigger(void);

/**
 * Add data_source to run_loop
 */
static void btstack_run_loop_ecos_add_data_source(btstack_data_source_t *ds){
    btstack_linked_list_add(&data_sources, (btstack_linked_item_t *) ds);
}

/**
 * Remove data_source from run loop
 */
static int btstack_run_loop_ecos_remove_data_source(btstack_data_source_t *ds){
    return btstack_linked_list_remove(&data_sources, (btstack_linked_item_t *) ds);
}


// set timer
static void btstack_run_loop_ecos_set_timer(btstack_timer_source_t *ts, uint32_t timeout_in_ms)
{
    uint32_t ticks = timeout_in_ms / SYSTEM_TICK_TO_MS_CONST;
    if (ticks == 0) ticks++;
    // time until next tick is < hal_tick_get_tick_period_in_ms() and we don't know, so we add one
    ts->timeout = cyg_current_time() + ticks; 
}

/**
 * Add timer to run_loop (keep list sorted)
 */
static void btstack_run_loop_ecos_add_timer(btstack_timer_source_t *ts){
    btstack_linked_item_t *it;
    for (it = (btstack_linked_item_t *) &timers; it->next ; it = it->next)
    {
        // don't add timer that's already in there
        if ((btstack_timer_source_t *) it->next == ts)
        {
            log_error( "btstack_run_loop_timer_add error: timer to add already in list!");
            return;
        }
        if (ts->timeout < ((btstack_timer_source_t *) it->next)->timeout) 
        {
            break;
        }
    }
    ts->item.next = it->next;
    it->next = (btstack_linked_item_t *) ts;
}

/**
 * Remove timer from run loop
 */
static int btstack_run_loop_ecos_remove_timer(btstack_timer_source_t *ts)
{
    return btstack_linked_list_remove(&timers, (btstack_linked_item_t *) ts);
}

static void btstack_run_loop_ecos_dump_timer(void)
{
#ifdef ENABLE_LOG_INFO 
    btstack_linked_item_t *it;
    int i = 0;
    for (it = (btstack_linked_item_t *) timers; it ; it = it->next){
        btstack_timer_source_t *ts = (btstack_timer_source_t*) it;
        log_info("timer %u, timeout %u\n", i, (unsigned int) ts->timeout);
    }
#endif
}

static uint32_t btstack_run_loop_ecos_get_time_ms(void)
{
    return cyg_current_time() * SYSTEM_TICK_TO_MS_CONST;
}

/**
 * Ecos timer callback 
 */
void btstack_ecos_timer_callback(cyg_handle_t alarmH, cyg_addrword_t data)
{
	btstack_run_loop_ecos_trigger();
	cyg_alarm_initialize(timer_alarmH, cyg_current_time() + SYSTEM_TIMER_COUNTS, 0);
}

uint32_t btstack_run_loop_ecos_get_ticks(void)
{
    return cyg_current_time();
}

uint32_t btstack_run_loop_ecos_ticks_for_ms(uint32_t time_in_ms)
{
    return time_in_ms/SYSTEM_TICK_TO_MS_CONST;
}

static void btstack_run_loop_ecos_init(void)
{
    data_sources = NULL;
    timers       = NULL;
	cyg_drv_mutex_init(&transfer_mx);
	cyg_drv_cond_init(&transfer_cond,&transfer_mx);

    system_clockH = cyg_real_time_clock();
    cyg_clock_to_counter(system_clockH, &timer_counterH);
    cyg_alarm_create(timer_counterH, btstack_ecos_timer_callback,
                            (cyg_addrword_t) NULL,
                            &timer_alarmH, &timer_alarm);
    
    cyg_alarm_initialize(timer_alarmH, cyg_current_time() + 10, 0);

}

static void btstack_run_loop_ecos_enable_data_source_callbacks(btstack_data_source_t * ds, uint16_t callback_types){
	ds->flags |= callback_types;
}

static void btstack_run_loop_ecos_disable_data_source_callbacks(btstack_data_source_t * ds, uint16_t callback_types){
	ds->flags &= ~callback_types;
}

void btstack_run_loop_ecos_execute_once(void) {
	btstack_data_source_t *ds;

	// process data sources
	btstack_data_source_t *next;
	for (ds = (btstack_data_source_t *) data_sources; ds != NULL ; ds = next){
		next = (btstack_data_source_t *) ds->item.next; // cache pointer to next data_source to allow data source to remove itself
		if (ds->flags & DATA_SOURCE_CALLBACK_POLL){
			ds->process(ds, DATA_SOURCE_CALLBACK_POLL);
		}

		if (ds->flags & DATA_SOURCE_CALLBACK_READ){
			ds->process(ds, DATA_SOURCE_CALLBACK_READ);
		}
	}

	uint32_t now = cyg_current_time();

	while (timers)
	{
		btstack_timer_source_t *ts = (btstack_timer_source_t *) timers;
		if (ts->timeout > now) break;
		btstack_run_loop_remove_timer(ts);
		ts->process(ts);
	}
}

static void btstack_run_loop_ecos_is_trigger()
{
	cyg_drv_mutex_lock(&transfer_mx);
	while(!trigger_event_received)
		cyg_drv_cond_wait(&transfer_cond);
	trigger_event_received = 0;
	cyg_drv_mutex_unlock(&transfer_mx);
}

static void btstack_run_loop_ecos_execute(void)
{
	while (1) 
	{
		btstack_run_loop_ecos_execute_once();	
		btstack_run_loop_ecos_is_trigger();
	}
}

void btstack_run_loop_ecos_trigger(void)
{
	trigger_event_received = 1;
	cyg_drv_cond_signal(&transfer_cond);
}

/**
 * Provide btstack_run_loop_ecos instance 
 */
const btstack_run_loop_t * btstack_run_loop_ecos_get_instance(void){
    return &btstack_run_loop_ecos;
}

static const btstack_run_loop_t btstack_run_loop_ecos = {
    &btstack_run_loop_ecos_init,
    &btstack_run_loop_ecos_add_data_source,
    &btstack_run_loop_ecos_remove_data_source,
    &btstack_run_loop_ecos_enable_data_source_callbacks,
    &btstack_run_loop_ecos_disable_data_source_callbacks,
    &btstack_run_loop_ecos_set_timer,
    &btstack_run_loop_ecos_add_timer,
    &btstack_run_loop_ecos_remove_timer,
    &btstack_run_loop_ecos_execute,
    &btstack_run_loop_ecos_dump_timer,
    &btstack_run_loop_ecos_get_time_ms,
};
