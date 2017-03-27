/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * Portions Copyright (c) 2011-2012 ARM Ltd.  All rights reserved.
 */

/*
   This code was adjusted to have eCos properties similar to i386 SMP.

   When unlocking, no CPU check is made. Therefore, any CPU can unlock it.

   There is no signaling between the lock and unlock.

   If the lock is already claimed, it just sets the value again and returns.

   However, it uses the CPU number to ease debugging.
*/

    .code 32
    .section ".text","ax"
    
    .global cpu_get_current

#define UNLOCKED 0xFF

// void hal_spinlock_lock(spinlock_t * lock, cyg_uint32 timeout)
    .global hal_spinlock_lock
    .func hal_spinlock_lock
hal_spinlock_lock:
    stmfd   sp!,{r2}
hal_spinlock_retry:
    ldrex   r1, [r0]
    cmp     r1, #UNLOCKED           // check if spinlock currently unlocked
    wfene                           // wait for an event signal
    bne     hal_spinlock_retry
    
    mrc     p15, 0, r1, c0, c0, 5   // get our CPU ID
    and     r1, r1, #3
    strex   r2, r1, [r0]            // attempt to grab lock by writing CPU number into spinlock
    cmp     r2, #0                  // check if the write was successful
    bne     hal_spinlock_retry      // if the write failed, start over
    
    dmb                             // Ensure that accesses to shared resource have completed
                                    // This makes sure the protected data is not touched until
                                    // the lock is taken
    
    ldmfd   sp!, {r2}
    bx      lr                      // return to caller

    .endfunc // spinlock_lock
    
    // cyg_bool hal_spinlock_trylock(spinlock_t * lock, cyg_uint32 timeout)
    .global hal_spinlock_trylock
    .func hal_spinlock_trylock
hal_spinlock_trylock:
    stmfd   sp!,{r2}

    ldrex   r1, [r0]
    cmp     r1, #UNLOCKED           // check if spinlock currently unlocked
    movne	r0,#1
    bne		hal_spinlock_trylock_return

    mrc     p15, 0, r1, c0, c0, 5   // get our CPU ID
    and     r1, r1, #3
    strex   r2, r1, [r0]            // attempt to grab lock by writing CPU number into spinlock
    cmp     r2, #0                  // check if the write was successful
    movne	r0,#1
    bne		hal_spinlock_trylock_return

    dmb                             // Ensure that accesses to shared resource have completed
                                    // This makes sure the protected data is not touched until
                                    // the lock is taken
    mov		r0,#0
hal_spinlock_trylock_return:
    ldmfd   sp!, {r2}
    bx      lr                      // return to caller

    .endfunc // spinlock_trylock

// cyg_uint32 hal_spinlock_unlock(spinlock_t * lock)
    .global hal_spinlock_unlock
    .func hal_spinlock_unlock
hal_spinlock_unlock:
    stmfd   sp!,{r1, r2}

    mrc     p15, 0, r1, c0, c0, 5   // get our CPU ID
    and     r1, r1, #3

    ldr     r2, [r0]                // read lock field of spinlock
    cmp     r1, r2                  // compare lock field with our CPU ID
    movne   r0, #1                  // doesn't match, so exit with failure
    bne    	hal_spinlock_unlock_return

    mov     r1, #UNLOCKED           // load unlocked value

    dmb                             // Ensure that accesses to shared resource have completed
                                    // This makes sure all data protected by the lock is seen
                                    // by all CPUs before we unlock.
    str     r1, [r0]                // Write into lock field of spinlock
                                    // A simple store is ok because only once CPU at a time will
                                    // release the lock
    dsb                             // Ensure that no instructions following the barrier execute until
                                    // all memory accesses prior to the barrier have completed.
                                    // This prevents any instructions from executing until the lock
                                    // is released, thus preventing the event from firing to soon.
    
    sev                             // send event to wake up other cores waiting on spinlock

    mov     r0, #0                  // return success
hal_spinlock_unlock_return:
    ldmfd   sp!, {r1, r2}
    bx      lr

    .endfunc

    .end
