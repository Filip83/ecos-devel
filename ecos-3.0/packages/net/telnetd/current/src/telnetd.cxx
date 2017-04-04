/* =================================================================
 *
 *      telnetd.c
 *
 *      A simple embedded TELNET server
 *
 * ================================================================= 
 * ####ECOSGPLCOPYRIGHTBEGIN####                                     
 * -------------------------------------------                       
 * This file is part of eCos, the Embedded Configurable Operating System.
 * Copyright (C) 2002, 2003 Free Software Foundation, Inc.           
 *
 * eCos is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 or (at your option) any later
 * version.                                                          
 *
 * eCos is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.                                                 
 *
 * You should have received a copy of the GNU General Public License 
 * along with eCos; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.     
 *
 * As a special exception, if other files instantiate templates or use
 * macros or inline functions from this file, or you compile this file
 * and link it with other works to produce a work based on this file,
 * this file does not by itself cause the resulting work to be covered by
 * the GNU General Public License. However the source code for this file
 * must still be made available in accordance with section (3) of the GNU
 * General Public License v2.                                        
 *
 * This exception does not invalidate any other reasons why a work based
 * on this file might be covered by the GNU General Public License.  
 * -------------------------------------------                       
 * ####ECOSGPLCOPYRIGHTEND####                                       
 * =================================================================
 * #####DESCRIPTIONBEGIN####
 * 
 *  Author(s):    Mike Jones <mike@proclivis.com>
 *  Data:         nickg@calivar.com, Andrew.lunn@ascom.ch
 *  Date:         2014-01-02
 *  Purpose:      
 *  Description:  
 *               
 * ####DESCRIPTIONEND####
 * 
 * =================================================================
 */

#include <pkgconf/system.h>
#include <pkgconf/memalloc.h>

#ifdef CYGPKG_KERNEL
#include <pkgconf/kernel.h>

#include <cyg/kernel/sched.hxx>         // Cyg_Scheduler::start()
#include <cyg/kernel/thread.hxx>        // Cyg_Thread
#include <cyg/kernel/thread.inl>
#include <cyg/kernel/sema.hxx>

#include <cyg/kernel/sched.inl>
#endif

#include <pkgconf/isoinfra.h>
#include <pkgconf/telnetd.h>

#include <sys/sockio.h>

#include <cyg/infra/cyg_trac.h>        /* tracing macros */
#include <cyg/infra/cyg_ass.h>         /* assertion macros */
#include <cyg/libc/stdio/stream.hxx>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <network.h>
#include <arpa/inet.h>

#include <cyg/telnetd/telnetd.hxx>


#if (CYGPKG_TELNETD_COMMAND_DLMALLOC)
#include <cyg/memalloc/dlmalloc.hxx>
#endif

/* ================================================================= */

#if 0
#define TELNETD_DIAG diag_printf
#else
#define TELNETD_DIAG(...)
#endif

/* ================================================================= */
/* Forward declarations
 */

__externC cyg_bool cyg_telnetd_send_response( FILE *fp, char *response );
 void cyg_telnetd_process_input( FILE *output, FILE *input, FILE *error,
		 char *input_data, void *user_data  );

/* ================================================================= */
/* Server sstate management
 */

#define TELNET_SE           240    // End of subnegotiation parameters.
#define TELNET_NOP          241    // No operation.
#define TELNET_DM           242    // The data stream portion of a Synch.
                                   // This should always be accompanied
                                   // by a TCP Urgent notification.
#define TELNET_BRK          243    // NVT character BRK.
#define TELNET_IP           244    // The function IP.
#define TELNET_AO           245    // The function AO.
#define TELNET_AYT          246    // The function AYT.
#define TELNET_EC           247    // The function EC.
#define TELNET_GA           249    // The GA signal.
#define TELNET_SB           250    // Indicates that what follows is
                                   // subnegotiation of the indicated
                                   // option.
#define TELNET_WILL         251    // Indicates the desire to begin
                                   // performing, or confirmation that
                                   // you are now performing, the
                                   // indicated option.
#define TELNET_WONT         252    // Indicates the refusal to perform,
                                   // or continue performing, the
                                   // indicated option.
#define TELNET_DO           253    // Indicates the request that the
                                   // other party perform, or
                                   // confirmation that you are expecting
                                   // the other party to perform, the
                                   // indicated option.
#define TELNET_DONT         254    // Indicates the demand that the
                                   // other party stop performing,
                                   // or confirmation that you are no
                                   // longer expecting the other party
                                   // to perform, the indicated option.
#define TELNET_IAC          255    // Data Byte 255.

#define TELNET_FA			3
#define TELNET_LM           34
#define TELNET_SLC			3

#define TELNET_MODE			1
#define TELNET_EDIT			1

#define STATE_NORMAL 	0
#define STATE_IAC		1
#define STATE_WILL		2
#define STATE_WONT		3
#define STATE_DO		4
#define STATE_DONT		5
#define STATE_SB		6
#define STATE_LM		7
#define STATE_MODE		8

#define STATE_SLC		9
#define STATE_SLC_F		10
#define STATE_SLC_M		11
#define STATE_SLC_C		12

/* ================================================================= */
/* Server socket address and file descriptor.
 */

static struct sockaddr_in server_address;

static int server_socket = -1;
#ifdef CYGPKG_NET_INET6
static int server_socket6 = -1;
static struct sockaddr_in6 server_address6;
#endif

/* ================================================================= */
/* Thread stacks, etc.
 */

static cyg_uint8 cyg_telnetd_stacks[CYGNUM_TELNETD_THREAD_COUNT]
                             [CYGNUM_HAL_STACK_SIZE_MINIMUM+
                              CYGNUM_TELNETD_SERVER_BUFFER_SIZE+
                              CYGNUM_TELNETD_THREAD_STACK_SIZE];
static cyg_handle_t cyg_telnetd_thread[CYGNUM_TELNETD_THREAD_COUNT];
static cyg_thread cyg_telnetd_thread_object[CYGNUM_TELNETD_THREAD_COUNT];
static int cyg_telnetd_client_socket[CYGNUM_TELNETD_THREAD_COUNT];

static char cyg_telnetd_thread_name[CYGNUM_TELNETD_THREAD_COUNT][12];

static FILE *cyg_telnetd_client_output[CYGNUM_TELNETD_THREAD_COUNT];
static FILE *cyg_telnetd_client_input[CYGNUM_TELNETD_THREAD_COUNT];
static FILE *cyg_telnetd_client_error[CYGNUM_TELNETD_THREAD_COUNT];
#ifdef CYGPKG_TELNETD_COMMAND_DLMALLOC
static cyg_uint8 cyg_telnetd_client_pool[CYGNUM_TELNETD_THREAD_COUNT][CYGPKG_TELNETD_COMMAND_DLMALLOC_SIZE];
static Cyg_Mempool_dlmalloc *cyg_telnetd_client_dlmalloc[CYGNUM_TELNETD_THREAD_COUNT];
#endif

static cyg_uint8 cyg_telnetd_monitor_stacks[CYGNUM_TELNETD_THREAD_COUNT]
                             [CYGNUM_HAL_STACK_SIZE_MINIMUM+
                              CYGNUM_TELNETD_SERVER_BUFFER_SIZE+
                              CYGNUM_TELNETD_THREAD_STACK_SIZE];
static cyg_handle_t cyg_telnetd_monitor_thread[CYGNUM_TELNETD_THREAD_COUNT];
static cyg_thread cyg_telnetd_thread_monitor_object[CYGNUM_TELNETD_THREAD_COUNT];

static char cyg_telnetd_thread_monitor_name[CYGNUM_TELNETD_THREAD_COUNT][24];

/* ================================================================= */
/* Command lookup table
 */

CYG_HAL_TABLE_BEGIN( cyg_telnetd_table, telnetd_table );
CYG_HAL_TABLE_END( cyg_telnetd_table_end, telnetd_table );

__externC cyg_telnetd_table_entry cyg_telnetd_table[];
__externC cyg_telnetd_table_entry cyg_telnetd_table_end[];

/* ================================================================= */
/* Common messages
 */

static const char *	prompt = ">";
static char *		delim = { " " };
static char cyg_telnetd_not_found[] = "Command Not Found";

/* ================================================================= */
/* Locks
 */

// A lock for opening and closing file descriptors. This prevents
// a deadlock that was discovered in the streams.
static cyg_mutex_t cyg_telnetd_fdmutex;

/* ================================================================= */
/* Common commands
 */

#ifdef CYGPKG_TELNETD_COMMAND_ECHO

cyg_bool cyg_telnetd_echo_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
	cyg_int32 pos;
	if (argc > 1)
	{
		cyg_uint32 total = 0;
		for (pos = 1; pos < argc; pos++)
			total += strlen(argv[pos]);
		total += 1;
		char *response = (char*) malloc(total);
		response[0] = '\0';
		for (pos = 1; pos < argc; pos++)
			strcat(response, argv[pos]);
		cyg_telnetd_send_response( output, response );
		// Does the ethernet layer copy this?
		free(response);
		return true;
	}
	else
	{
		// Space forces send.
		cyg_telnetd_send_response( output, " " );
		return true;
	}
}

CYG_TELNETD_TABLE_ENTRY( __telnet_echo, "echo*", cyg_telnetd_echo_handler, (void*) 0 );
#endif

#ifdef CYGPKG_TELNETD_COMMAND_SH

#include <cyg/sh/sh.h>

CYG_TELNETD_TABLE_ENTRY( __sh_pwd, "pwd*", cyg_sh_pwd_handler, (void*) 0 );
CYG_TELNETD_TABLE_ENTRY( __sh_ls, "ls*", cyg_sh_ls_handler, (void*) 0 );
CYG_TELNETD_TABLE_ENTRY( __sh_cd, "cd*", cyg_sh_cd_handler, (void*) 0 );
CYG_TELNETD_TABLE_ENTRY( __sh_mkdir, "mkdir*", cyg_sh_mkdir_handler, (void*) 0 );
CYG_TELNETD_TABLE_ENTRY( __sh_rmdir, "rmdir*", cyg_sh_rmdir_handler, (void*) 0 );
CYG_TELNETD_TABLE_ENTRY( __sh_rm, "rm*", cyg_sh_rm_handler, (void*) 0 );
CYG_TELNETD_TABLE_ENTRY( __sh_mv, "mv*", cyg_sh_mv_handler, (void*) 0 );
CYG_TELNETD_TABLE_ENTRY( __sh_hd, "hd*", cyg_sh_hd_handler, (void*) 0 );

CYG_TELNETD_TABLE_ENTRY( __sh_heap, "heap*", cyg_sh_heap_handler, (void*) 0 );
CYG_TELNETD_TABLE_ENTRY( __sh_task, "task*", cyg_sh_task_handler, (void*) 0 );
CYG_TELNETD_TABLE_ENTRY( __network_task, "network*", cyg_sh_network_handler, (void*) 0 );

#endif

#ifdef CYGPKG_TELNETD_COMMAND_LUA

CYG_TELNETD_TABLE_ENTRY( __telnet_lua, "lua*", cyg_sh_lua_handler, (void*)0 );

#endif

/* ================================================================= */
/* Simple pattern matcher for commands
 *
 * This performs a simple pattern match between the given name and the
 * pattern. At present the only matching supported is either exact, or
 * if the pattern ends in * then that matches all remaining
 * characters. At some point we might want to implement a more
 * complete regular expression parser here.
 */

static cyg_bool match( char *name, char *pattern )
{
    while( *name != 0 && *pattern != 0 && *name == *pattern )
        name++, pattern++;

    if( *name == 0 && *pattern == 0 )
        return true;

    if( *pattern == '*' )
        return true;

    return false;
}


/* ================================================================= */
/* Main processing function                                          */
/*                                                                   */
/* Reads the TELNET command, look it up in the table and calls the   */
/* handler.                                                          */

static void cyg_telnetd_process(
		int client_socket,
		struct sockaddr *client_address,
		FILE *output,
		FILE *input,
		FILE *error,
		void *user_data)
{
    int calen = sizeof(*client_address);
    char name[64];
    char port[10];

    uint16_t			state = STATE_NORMAL;
    uint8_t				buffer[CYGNUM_TELNETD_SERVER_BUFFER_SIZE];
    uint16_t 			length = CYGNUM_TELNETD_SERVER_BUFFER_SIZE;
	uint8_t 			c;
	uint8_t 			*p;
	ssize_t				qty;
	uint8_t				f, m, ch;
	cyg_bool			linemode = false;

    getnameinfo(client_address, calen, name, sizeof(name), 
                port, sizeof(port), NI_NUMERICHOST|NI_NUMERICSERV);
    TELNETD_DIAG("Connection from %s[%s]\n",name,port);

	p = buffer;
	length = CYGNUM_TELNETD_SERVER_BUFFER_SIZE - 1;

	while (1)
	{

		qty = read( client_socket, &c, 1 );

		if (qty <= 0)
		{
			fclose(output);
			fclose(input);
			fclose(error);
			return;
		}

		switch (state)
		{
		case STATE_IAC:
			if (c == TELNET_IAC)
			{
				return;
			}
			else
			{
				switch (c)
				{
				case TELNET_WILL:
					TELNETD_DIAG("command: WILL ");
					state = STATE_WILL;
					break;
				case TELNET_WONT:
					TELNETD_DIAG("command: WONT ");
					state = STATE_WONT;
					break;
				case TELNET_DO:
					TELNETD_DIAG("command: DO ");
					state = STATE_DO;
					break;
				case TELNET_DONT:
					TELNETD_DIAG("command: DONT ");
					state = STATE_DONT;
					break;
				case TELNET_SB:
					TELNETD_DIAG("command: SB");
					state = STATE_SB;
					break;
				case TELNET_SE:
					TELNETD_DIAG("command: SE\n");
					state = STATE_NORMAL;
					break;
				default:
					TELNETD_DIAG("command: default\n");
					state = STATE_NORMAL;
					break;
				}
			}
			break;
		case STATE_WILL:
			TELNETD_DIAG(" 0x%x\n", c);
			switch (c)
			{
			case TELNET_LM :
				TELNETD_DIAG("reply: DO 0x%x\n", c);
				cyg_telnetd_send_data( client_socket, TELNET_DO, c );
				linemode = true;
				char command[8];
				command[0] = TELNET_IAC;
				command[1] = TELNET_SB;
				command[2] = TELNET_LM;
				command[3] = TELNET_MODE;
				command[4] = TELNET_EDIT;
				command[5] = TELNET_IAC;
				command[6] = TELNET_SE;
				command[7] = 0x00;
				cyg_telnetd_send_data_array( client_socket, command );
			    telnet_begin( client_socket );
				break;
			case TELNET_GA :
				TELNETD_DIAG("reply: DO 0x%x\n", c);
				break;
			default:
				TELNETD_DIAG("reply: DONT 0x%x\n", c);
				cyg_telnetd_send_data( client_socket, TELNET_DONT, c );
				break;
			}
			state = STATE_NORMAL;
			break;
		case STATE_WONT:
			TELNETD_DIAG(" 0x%x\n", c);
			TELNETD_DIAG("reply: DONT 0x%x\n", c);
			cyg_telnetd_send_data( client_socket, TELNET_DONT, c );
			state = STATE_NORMAL;
			break;
		case STATE_DO:
			TELNETD_DIAG(" 0x%x\n", c);
			switch(c)
			{
			case TELNET_LM :
				TELNETD_DIAG("reply: DO 0x%x\n", c);
				cyg_telnetd_send_data( client_socket, TELNET_DO, c );
				break;
			default:
				TELNETD_DIAG("reply: DONT 0x%x\n", c);
				cyg_telnetd_send_data( client_socket, TELNET_DONT, c );
				break;
			}
			state = STATE_NORMAL;
			break;
		case STATE_DONT:
			TELNETD_DIAG(" 0x%x\n", c);
			TELNETD_DIAG("reply: DONT 0x%x\n", c);
			cyg_telnetd_send_data( client_socket, TELNET_DONT, c );
			state = STATE_NORMAL;
			break;
		case STATE_SB:
			TELNETD_DIAG(" 0x%x\n", c);
			switch(c)
			{
			case TELNET_LM:
				state = STATE_LM;
				break;
			default:
				// This is a failure
				state = STATE_NORMAL;
				break;
			}
			break;
		case STATE_LM:
			TELNETD_DIAG(" 0x%x\n", c);
			switch (c)
			{
			case TELNET_SLC:
				state = STATE_SLC;
				break;
			case TELNET_MODE:
				state = STATE_MODE;
				break;
			default:
				// Pathological case.
				state = STATE_NORMAL;
				break;
			}
			break;
			case STATE_SLC:
				switch(c)
				{
				case TELNET_IAC:
					state = STATE_IAC;
					break;
				default:
					f = c;
					state = STATE_SLC_F;
					break;
				}
				break;
			case STATE_MODE:
				switch(c)
				{
				case TELNET_IAC:
					state = STATE_IAC;
					break;
				default:
					// Grab mode.
					state = STATE_MODE;
					break;
				}
				break;
		case STATE_SLC_F:
			m = c;
			state = STATE_SLC_M;
			break;
		case STATE_SLC_M:
			ch = c;
			TELNETD_DIAG("0x%x 0x%x 0x%x\n", f, m, ch);
			if (c != 0xff) // If 255, there will be two of these
				state = STATE_SLC;
			else
			{
				qty = read( client_socket, &c, 1 );
				state = STATE_SLC;
			}
			break;
		case STATE_NORMAL:
			if(c == TELNET_IAC)
			{
				state = STATE_IAC;
			}
			else
			{
				TELNETD_DIAG("command: ");
				while (qty != 0 && c != 0 && c != '\n' && c != '\r')
				{
					if (length > 0)
					{
						*p = c;
						p++;
						length--;
					}
					qty = read( client_socket, &c, 1);
				}
				*p = '\0';
				TELNETD_DIAG("%s\n", buffer);
				if (!strcmp ((const char*)buffer, "quit") ||
						!strcmp((const char*)buffer, "q") ||
						!strcmp((const char*)buffer, "qu") ||
						!strcmp((const char*)buffer, "qui"))
				{
					fclose(output);
					fclose(input);
					fclose(error);
					return;
				}
				if (strlen((const char*)buffer) > 0)
				{
					cyg_telnetd_process_input(output, input, error, (char*)buffer, user_data);
					telnet_end (client_socket);
				}
				if (strlen((const char*)buffer) == 0 && c == '\r')
				{
					if (!linemode)
						cyg_telnetd_send_response( output, " " );
					telnet_end (client_socket);
				}
				p = buffer;
				length = CYGNUM_TELNETD_SERVER_BUFFER_SIZE - 1;
			}
			break;
		}

	}
}

/* ================================================================= */
/* Monitor server
 *
 * This watches for a lost connection and kills the thread.
 * It is somewhat dangerous because there is no attempt to
 * unlock any mutex, free memory, etc.
 *
 */
static void cyg_telnetd_server( cyg_addrword_t arg );
static void cyg_telnetd_monitor_server( cyg_addrword_t arg )
{
	int index = (int) arg;
    char name[20] = "";
	int data;
	Cyg_StdioStream *stream;
	cyg_bool success;
	cyg_uint32 tries;
	do
	{
		cyg_thread_delay(100);
		if (cyg_telnetd_client_socket[index] > 0)
		{
			ioctl(cyg_telnetd_client_socket[index], SIOCANTRCVMORE, (CYG_ADDRWORD)&data);
			if (data)
			{
				// Try to kill it.
				cyg_thread_kill(cyg_telnetd_thread[index]);
				// Wait for it to exit.
				cyg_thread_info info;
				info.state = 0;
				cyg_uint16 timeout = 1000;
				while (info.state != Cyg_Thread::EXITED && timeout > 0)
				{
					cyg_thread_get_info(
						cyg_telnetd_thread[index],
						cyg_thread_get_id(cyg_telnetd_thread[index]),
						&info);
					timeout++;
					cyg_thread_delay(1);
				}
				// Delete its resources if it exited. If not, this thread
				// is going to hang around and things are going to get
				// into trouble. Basically the thread will not get restarted
				// and the system will run out of threads if it happens too
				// many times. This should never happen, but thread kill is
				// dangerous so this is a precaution.
				if (info.state == Cyg_Thread::EXITED)
				{
					if(cyg_thread_delete(cyg_telnetd_thread[index]))
					{
						cyg_mutex_lock(&cyg_telnetd_fdmutex);
						stream = (Cyg_StdioStream *)cyg_telnetd_client_output[index];
						success = false;
						tries = 100;
						while (!success && tries-- > 0)
						{
							success = stream->trylock_me();
							cyg_thread_delay(1);
						}
						if (!success)
						{
							tries = 10000;
							while (!success && tries-- > 0)
							{
								success = stream->trylock_me();
							}
						}
						if (success)
							stream->unlock_me();
						else
						{
							// We hope that if we are here the lock is from the
							// dead thread and we unlock anyway. This will fail
							// if asertions are enabled.
							stream->unlock_me();
							// Just in case, we allow other threads to continue
							// for awhile until before we let the code continue
							// and interact with the lock to minimize interactions.
							cyg_thread_delay(10);
						}
						// The unlock should apply to all three streams because
						// they share the same lock.
						fclose(cyg_telnetd_client_output[index]);
						fclose(cyg_telnetd_client_input[index]);
						fclose(cyg_telnetd_client_error[index]);
						cyg_mutex_unlock(&cyg_telnetd_fdmutex);
						close(cyg_telnetd_client_socket[index]);
						// Prevent closing again.
						cyg_telnetd_client_socket[index] = 0;
		#ifdef CYGPKG_TELNETD_COMMAND_DLMALLOC
						free(cyg_telnetd_client_dlmalloc[index]);
		#endif
						sprintf(name, "TELNETD %d", index);
						cyg_thread_create( CYGNUM_TELNETD_THREAD_PRIORITY,
										   cyg_telnetd_server,
										   index,
										   name,
										   &cyg_telnetd_stacks[index][0],
										   sizeof(cyg_telnetd_stacks[index]),
										   &cyg_telnetd_thread[index],
										   &cyg_telnetd_thread_object[index]
							);
						cyg_thread_resume( cyg_telnetd_thread[index] );
					}
				}
			}
		}
	}while(1);
}

/* ================================================================= */
/* Main TELNET server
 *
 * This just loops, collects client connections, and calls the main
 * process function on the connects*/

static void cyg_telnetd_server( cyg_addrword_t arg )
{
	int index = (int) arg;

	do
	{
		int client_socket;
		struct sockaddr client_address;
		socklen_t calen = sizeof(client_address);
		fd_set readfds;
		int n;
		char name[20] = "";

		/* Wait for a connection.
		 */
		FD_ZERO(&readfds);
		FD_SET(server_socket, &readfds);
#ifdef CYGPKG_NET_INET6
		FD_SET(server_socket6, &readfds);
		n = (server_socket > server_socket6 ? server_socket : server_socket6) + 1;
#else
		n = server_socket + 1;
#endif
		select(n,&readfds,NULL,NULL,NULL);
		if (FD_ISSET(server_socket, &readfds)) {
			client_socket = accept( server_socket, &client_address, &calen );
		}

#ifdef CYGPKG_NET_INET6
		if (FD_ISSET(server_socket6, &readfds)) {
		  client_socket = accept( server_socket6, &client_address, &calen );
		  cyg_telnetd_process(client_socket, &client_address);
		}
#endif
		if (FD_ISSET(server_socket, &readfds)
#ifdef CYGPKG_NET_INET6
		|| FD_ISSET(server_socket6, &readfds)
#endif
		) {
			cyg_telnetd_client_socket[index] = client_socket;
			cyg_mutex_lock(&cyg_telnetd_fdmutex);
			cyg_telnetd_client_output[index] = fdopen(client_socket, "w");
			cyg_telnetd_client_input[index] = fdopen(client_socket, "r");
			cyg_telnetd_client_error[index] = fdopen(client_socket, "w");
			cyg_mutex_unlock(&cyg_telnetd_fdmutex);
			void *user_data = NULL;
#ifdef CYGPKG_TELNETD_COMMAND_DLMALLOC
			cyg_telnetd_client_dlmalloc[index] = new Cyg_Mempool_dlmalloc(cyg_telnetd_client_pool[index], CYGPKG_TELNETD_COMMAND_DLMALLOC_SIZE);
			user_data = (void*)cyg_telnetd_client_dlmalloc[index];
#endif
			cyg_telnetd_process(client_socket, &client_address,
					cyg_telnetd_client_output[index],
					cyg_telnetd_client_input[index],
					cyg_telnetd_client_error[index],
					user_data);
#ifdef CYGPKG_TELNETD_COMMAND_DLMALLOC
			free(cyg_telnetd_client_dlmalloc[index]);
#endif
		}

	} while(1);
}

/* ================================================================= */
/* Initialization thread
 *
 * Optionally delay for a time before getting the network
 * running. Then create and bind the server socket and put it into
 * listen mode. Spawn any further server threads, then enter server
 * mode.
 */

static void cyg_telnetd_init(cyg_addrword_t arg)
{
    int i;
    int err = 0;
    for (i = 0; i < CYGNUM_TELNETD_THREAD_COUNT; i++)
    {
    	cyg_telnetd_thread_name[i][0] = '\0';
    	cyg_telnetd_thread_monitor_name[i][0] = '\0';
        cyg_telnetd_client_socket[i] = 0;
    }

    /* Delay for a configurable length of time to give the application
     * a chance to get going, or even complete, without interference
     * from the TELNETD.
     */
    if( CYGNUM_TELNETD_SERVER_DELAY > 0 )
    {
        cyg_thread_delay( CYGNUM_TELNETD_SERVER_DELAY );
    }
    
    server_address.sin_family = AF_INET;
    server_address.sin_len = sizeof(server_address);
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_port = htons(CYGNUM_TELNETD_SERVER_PORT);
#ifdef CYGPKG_NET_INET6
    server_address6.sin6_family = AF_INET6;
    server_address6.sin6_len = sizeof(server_address6);
    server_address6.sin6_addr = in6addr_any;
    server_address6.sin6_port = htons(CYGNUM_TELNETD_SERVER_PORT);
#endif 
    /* Get the network going. This is benign if the application has
     * already done this.
     */
    init_all_network_interfaces();

    /* Create and bind the server socket.
     */
    server_socket = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP );
    CYG_ASSERT( server_socket > 0, "Socket create failed");

    err = bind( server_socket, (struct sockaddr *)&server_address,
                sizeof(server_address) );
    CYG_ASSERT( err == 0, "bind() returned error");

    err = listen( server_socket, SOMAXCONN );
    CYG_ASSERT( err == 0, "listen() returned error" );
#ifdef CYGPKG_NET_INET6
    server_socket6 = socket( AF_INET6, SOCK_STREAM, IPPROTO_TCP );
    CYG_ASSERT( server_socket6 > 0, "Socket AF_INET6 create failed");

    err = bind( server_socket6, (struct sockaddr *)&server_address6,
                sizeof(server_address6) );
    CYG_ASSERT( err == 0, "bind(AF_INET6) returned error");

    err = listen( server_socket6, SOMAXCONN );
    CYG_ASSERT( err == 0, "listen(AF_INET6) returned error" );
#endif

    /* If we are configured to have more than one server thread,
     * create them now.
     */
    for( i = 1; i < CYGNUM_TELNETD_THREAD_COUNT; i++ )
    {
		sprintf(cyg_telnetd_thread_monitor_name[i], "TELNETD MONITOR %d", i);
		// A little more priority to make sure it can get time to check
		// the telnet thread in case it is in an infinite loop.
		cyg_thread_create( CYGNUM_TELNETD_THREAD_PRIORITY - 1,
						   cyg_telnetd_monitor_server,
						   i,
						   cyg_telnetd_thread_monitor_name[i],
						   &cyg_telnetd_monitor_stacks[i][0],
						   sizeof(cyg_telnetd_monitor_stacks[i]),
						   &cyg_telnetd_monitor_thread[i],
						   &cyg_telnetd_thread_monitor_object[i]
			);
		cyg_thread_resume( cyg_telnetd_monitor_thread[i] );

    	sprintf(cyg_telnetd_thread_name[i], "TELNETD %d", i);
        cyg_thread_create( CYGNUM_TELNETD_THREAD_PRIORITY,
                           cyg_telnetd_server,
                           i,
                           cyg_telnetd_thread_name[i],
                           &cyg_telnetd_stacks[i][0],
                           sizeof(cyg_telnetd_stacks[i]),
                           &cyg_telnetd_thread[i],
                           &cyg_telnetd_thread_object[i]
            );
        cyg_thread_resume( cyg_telnetd_thread[i] );
    }

	sprintf(cyg_telnetd_thread_monitor_name[0], "TELNETD MONITOR %d", 0);
	cyg_thread_create( CYGNUM_TELNETD_THREAD_PRIORITY - 1,
					   cyg_telnetd_monitor_server,
					   0,
					   "TELNETD MONITOR 0",
					   &cyg_telnetd_monitor_stacks[0][0],
					   sizeof(cyg_telnetd_monitor_stacks[0]),
					   &cyg_telnetd_monitor_thread[0],
					   &cyg_telnetd_thread_monitor_object[0]
		);
	cyg_thread_resume( cyg_telnetd_monitor_thread[0] );

    /* Now go be a server ourself.
     */
    cyg_telnetd_server(arg);
}

/* ================================================================= */
/* System initializer
 *
 * This is called from the static constructor in init.cxx. It spawns
 * the main server thread and makes it ready to run. It can also be
 * called explicitly by the application if the auto start option is
 * disabled.
 */

__externC void cyg_telnetd_startup(void)
{
	cyg_mutex_init(&cyg_telnetd_fdmutex);

    cyg_thread_create( CYGNUM_TELNETD_THREAD_PRIORITY,
                       cyg_telnetd_init,
                       0,
                       "TELNETD 0",
                       &cyg_telnetd_stacks[0][0],
                       sizeof(cyg_telnetd_stacks[0]),
                       &cyg_telnetd_thread[0],
                       &cyg_telnetd_thread_object[0]
        );
    cyg_thread_resume( cyg_telnetd_thread[0] );
}

/* ================================================================= */
/*  TELNET protocol handling
 *
 * cyg_telnet_start() and cyg_telnet_finish() manage the prompt.
 */

__externC void cyg_telnet_start( int fd )
{
    write( fd, prompt, strlen(prompt) );
}

__externC void cyg_telnet_finish( int fd )
{
    write( fd, prompt, strlen(prompt) );
}
    
/* ----------------------------------------------------------------- */
/* Send data
 *
 */

__externC cyg_bool cyg_telnetd_send_data( int fd, char command, char value )
{
    char send_data[3];
    send_data[0] = TELNET_IAC;
	send_data[1] = command;
	send_data[2] = value;
	write(fd, send_data, 3);
    return true;
}

__externC cyg_bool cyg_telnetd_send_data_array( int fd, char *data )
{
	write(fd, data, strlen(data));
    return true;
}

__externC cyg_bool cyg_telnetd_send_response( FILE *fp, char *response )
{
	if (strlen(response) > 0)
	{
		cyg_mutex_lock(&cyg_telnetd_fdmutex);
    	fwrite( response, 1, strlen(response), fp );
        fwrite( "\r\n", 1, 2, fp );
        fflush( fp );
		cyg_mutex_unlock(&cyg_telnetd_fdmutex);
	}

    return true;
}

void cyg_telnetd_process_input( FILE *output, FILE *input, FILE *error, char *input_data, void *user_data )
{
    cyg_telnetd_table_entry *entry = cyg_telnetd_table;
    cyg_bool success = false;

	char 		*save;
	char 		*arguement;
	char		*pos1, *pos2;
	int32_t 	argc1;
	char 		**argv1;
	int32_t 	argc2;
	char 		**argv2;
	int16_t     i, j;

	// First pass tokens using delimiters.
	argc1 = 0;
	argv1 = NULL;
	arguement = strtok_r(input_data, delim, &save);
	do {
		argc1++;
		argv1 = (char **) realloc(argv1, argc1 * sizeof(char **));
		(*(argv1 + argc1 - 1)) = arguement;
		arguement = strtok_r(NULL, delim, &save);
	} while (arguement != NULL);

	// Second pass combines tokens that demarcate strings.
	argc2 = 0;
	argv2 = (char **) malloc(argc1 * sizeof(char **));
	for (i = 0; i < argc1; i++)
	{
		if (strncmp(argv1[i], "\"", 1) == 0)
		{
			argv2[argc2] = (char *) malloc(strlen(argv1[i]));
			strcpy(argv2[argc2], argv1[i] + 1);
			if (strncmp(argv2[argc2] + strlen(argv2[argc2]) - 2, "\\\"", 2) != 0 &&
       				strncmp(argv2[argc2] + strlen(argv2[argc2]) - 1, "\"", 1) == 0)
			{
				argv2[argc2][strlen(argv2[argc2]) - 1] = '\0';
			}
			else
			{
				for (j = i + 1; j < argc1; j++)
				{
					argv2[argc2] = (char *) realloc(argv2[argc2], strlen(argv2[argc2]) + strlen(argv1[j]) + 2);
					strcat(argv2[argc2], " ");
					strcat(argv2[argc2], argv1[j]);
					if (strncmp(argv2[argc2] + strlen(argv2[argc2]) - 2, "\\\"", 2) != 0 &&
							strncmp(argv2[argc2] + strlen(argv2[argc2]) - 1, "\"", 1) == 0)
					{
						argv2[argc2][strlen(argv2[argc2]) - 1] = '\0';
						pos1 = pos2 = argv2[argc2];
						while (*pos2 != '\0')
						{
							if (strncmp(pos2, "\\\"", 2) == 0)
								pos2++;
							*pos1 = *pos2;
							pos1++;
							pos2++;
						}
						*pos1 = '\0';
						i = j;
						argc2++;
						break;
					}
				}
			}
			}
 		else
 		{
			argv2[argc2] = (char *) malloc(strlen(argv1[i]) + 1);
			strcpy(argv2[argc2], argv1[i]);
			argc2++;
 		}
	}

	free(argv1);

    TELNETD_DIAG("table: %08x...%08x\n",cyg_telnetd_table, cyg_telnetd_table_end);

    /* Now scan the table for a matching entry. If we find one
     * call the handler routine. If that returns true then we
     * terminate the scan, otherwise we keep looking.
     */
    while( entry != cyg_telnetd_table_end )
    {
        TELNETD_DIAG("try %08x: %s\n", entry, entry->pattern);

        if( match( input_data, entry->pattern ) )
        {
            TELNETD_DIAG("calling %08x: %s\n", entry, entry->pattern);

        	if (strlen(argv2[0]) > 0)
        	{
        		success = entry->handler(
        				output, input, error, input_data, argc2, argv2, entry->arg, user_data );

        		if (success)
        			break;
        	}
        }
        entry++;
    }

	for (i = 0; i < argc2; i++)
		free(argv2[i]);
	free(argv2);

    /* If we failed to find a match in the table, send a "not
     * found" response.
     */
    if( !success )
    {
        TELNETD_DIAG("Command Not Found: %s\r\n", input);
        cyg_telnetd_send_response( output, cyg_telnetd_not_found );
    }
}

/* ----------------------------------------------------------------- */
/* end of telnetd.c                                                    */
