/* =================================================================
 *
 *      sh.c
 *
 *      A simple embedded shell
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
 *  Date:         2014-01-15
 *  Purpose:      
 *  Description:  
 *               
 * ####DESCRIPTIONEND####
 * 
 * =================================================================
 */

/************************************************************************
 * Notes:
 *
 * - An obvious improvement would be to check lengths of all string
 *   operations to prevent overflow.
 *
 ************************************************************************/

#include <pkgconf/system.h>
#include <pkgconf/net.h>
#include <pkgconf/sh.h>
#if CYGPKG_TELNETD
#include <pkgconf/telnetd.h>
#endif

#include <cyg/infra/cyg_trac.h>        /* tracing macros */
#include <cyg/infra/cyg_ass.h>         /* assertion macros */

#include <cyg/io/file.h>
#include <cyg/fileio/dirent.h>
#include <cyg/fileio/fileio.h>

#include <fcntl.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>

#include <cyg/sh/sh.h>

#if CYGPKG_SH_COMMAND_LUA
#include <cyg/lua/lua.h>
#include <cyg/lua/lstate.h>
#include <cyg/lua/lauxlib.h>
#endif
#if CYGPKG_TELNETD_COMMAND_DLMALLOC
#include <cyg/memalloc/dlmalloc.hxx>
#endif

static const char *month_table[12] = {
    "Jan",
    "Feb",
    "Mar",
    "Apr",
    "May",
    "Jun",
    "Jul",
    "Aug",
    "Sep",
    "Oct",
    "Nov",
    "Dec"
};

#define SHELL_BUF_SIZE 132

static const shell_buf_size = SHELL_BUF_SIZE;
static char	shell_buf[SHELL_BUF_SIZE + 1];

/* ================================================================= */

#if 0
#define SH_DIAG diag_printf
#else
#define SH_DIAG(...)
#endif

cyg_bool cyg_sh_pwd_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
	Cyg_ErrNo 		retval;
	struct dirent 	dir_data;
	struct dirent	*dir_result;
	DIR 			*dir;
	char			file_time[60];

	if (argc != 1)
	{
		cyg_telnetd_send_response(output, "Incorrect number of parameters.");
	}
	else
	{
		if (strcmp(argv[0], "pwd") != 0)
			return false;

		getcwd(shell_buf, shell_buf_size);
		if (strlen(shell_buf) == 0)
			strcpy(shell_buf, "/disk0");

		cyg_telnetd_send_response(output, shell_buf);
	}
	return true;
}

cyg_bool cyg_sh_ls_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
	Cyg_ErrNo 		retval;
	struct dirent 	dir_data;
	struct dirent	*dir_result;
	DIR 			*dir;
	char			file_time[60];

	if (argc != 1 && argc != 2)
	{
		cyg_telnetd_send_response(output, "Incorrect number of parameters.");
	}
	else
	{
		if (strcmp(argv[0], "ls") != 0)
			return false;

		if (argc == 1)
		{
			getcwd(shell_buf, shell_buf_size);
			if (strlen(shell_buf) == 0)
				strcpy(shell_buf, "/disk0");
			dir = opendir(shell_buf);
		}
		else
			dir = opendir(argv[1]);
		if (dir == 0)
		{
			sprintf(shell_buf, "opendir() return -1 %d: %s.", errno, strerror(errno));
			cyg_telnetd_send_response(output, shell_buf);
		}
		else
		{
			while (-1 != (retval = readdir_r(dir, &dir_data, &dir_result)))
			{
				if (dir_result != NULL)
				{
					struct stat st;
					stat(dir_result->d_name, &st);
					ctime_r(&st.st_ctime, file_time);
					if (st.st_size >= 0)
					{
						char prefix = '-';
						// For some reason there is a 0x92 at the beginning.
						if ((st.st_mode & 0xFFFF) == S_IFDIR)
							prefix = 'd';
						struct tm *s_time = gmtime(&st.st_mtime);
						int current_year = s_time->tm_year;
						if (s_time->tm_year == current_year)
							sprintf(shell_buf, "%c-rw-rw-rw-   1 user     ftp  %11ld %s %02i %02i:%02i %s", prefix, st.st_size, month_table[s_time->tm_mon], s_time->tm_mday, s_time->tm_hour, s_time->tm_min, dir_result->d_name);
						else
							sprintf(shell_buf, "%c-rw-rw-rw-   1 user     ftp  %11ld %s %02i %5i %s", prefix, st.st_size, month_table[s_time->tm_mon], s_time->tm_mday, s_time->tm_year + 1900, dir_result->d_name);
						cyg_telnetd_send_response(output, shell_buf);
					}
				}
				else
					break;
			}
			if (-1 == retval)
			{
				sprintf(shell_buf, "readdir_r() return -1 %d: %s.", errno, strerror(errno));
				cyg_telnetd_send_response(output, shell_buf);
			}

			retval = closedir(dir);
			if (-1 == retval)
			{
				sprintf(shell_buf, "closedir() return -1 %d: %s.", errno, strerror(errno));
				cyg_telnetd_send_response(output, shell_buf);
			}
		}
	}
	return true;
}

cyg_bool cyg_sh_cd_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
	Cyg_ErrNo 		retval;
	struct dirent 	dir_data;
	struct dirent	*dir_result;
	DIR 			*dir;
	struct 			stat st;

	if (argc != 2)
	{
		cyg_telnetd_send_response(output, "Incorrect number of parameters.");
	}
	else
	{
		if (strcmp(argv[0], "cd") != 0)
			return false;

		getcwd(shell_buf, shell_buf_size);
		if (strlen(shell_buf) == 0)
			strcpy(shell_buf, "/disk0");
		dir = opendir(shell_buf);
		if (dir == 0)
		{
			sprintf(shell_buf, "opendir() return -1 %d: %s.", errno, strerror(errno));
			cyg_telnetd_send_response(output, shell_buf);
		}
		else
		{
			retval = closedir(dir);
			if (-1 == retval)
			{
				sprintf(shell_buf, "closedir() return -1 %d: %s.", errno, strerror(errno));
				cyg_telnetd_send_response(output, shell_buf);
			}
			else
			{
				strncat(shell_buf, "/", 1);
				strncat(shell_buf, argv[1], shell_buf_size);
				dir = opendir(shell_buf);
				if (dir == 0)
				{
					sprintf(shell_buf, "opendir() return -1 %d: %s.", errno, strerror(errno));
					cyg_telnetd_send_response(output, shell_buf);
				}
				else
				{
					if(stat(shell_buf, &st) == 0)
					{
						chdir(shell_buf);
						cyg_telnetd_send_response(output, "");
					}
					else
					{
						cyg_telnetd_send_response(output, "Directory does not exist.");
					}
				}
			}

			retval = closedir(dir);
			if (-1 == retval)
			{
				sprintf(shell_buf, "closedir() return -1 %d: %s.", errno, strerror(errno));
				cyg_telnetd_send_response(output, shell_buf);
			}
		}
	}
	return true;
}

cyg_bool cyg_sh_mkdir_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
	Cyg_ErrNo 		retval;
	struct dirent 	dir_data;
	struct dirent	*dir_result;
	DIR 			*dir;
	struct 			stat st;

	if (argc != 2)
	{
		cyg_telnetd_send_response(output, "Incorrect number of parameters.");
	}
	else
	{
		if (strcmp(argv[0], "mkdir") != 0)
			return false;

		getcwd(shell_buf, shell_buf_size);
		dir = opendir(shell_buf);
		if (dir == 0)
		{
			sprintf(shell_buf, "opendir() return -1 %d: %s.", errno, strerror(errno));
			cyg_telnetd_send_response(output, shell_buf);
		}
		else
		{
			retval = closedir(dir);
			if (-1 == retval)
			{
				sprintf(shell_buf, "closedir() return -1 %d: %s.", errno, strerror(errno));
				cyg_telnetd_send_response(output, shell_buf);
			}
			else
			{
				if (strlen(shell_buf) == 0)
					strcpy(shell_buf, "/disk0");
				strncat(shell_buf, "/", 1);
				strncat(shell_buf, argv[1], shell_buf_size);

				retval = mkdir(shell_buf, 0x777);
				if (EEXIST == retval)
					cyg_telnetd_send_response(output, "Directory already exists.");
				else if (0 != retval)
				{
					sprintf(shell_buf, "mkdir() return -1 %d: %s.", errno, strerror(errno));
					cyg_telnetd_send_response(output, shell_buf);
				}
				else
					cyg_telnetd_send_response(output, "");
			}
		}
	}
	retval = cyg_fs_setinfo("/disk0", FS_INFO_SYNC, NULL, NULL);
	return true;
}

cyg_bool cyg_sh_rmdir_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
	Cyg_ErrNo 		retval;
	struct dirent 	dir_data;
	struct dirent	*dir_result;
	DIR 			*dir;
	struct 			stat st;

	if (argc != 2)
	{
		cyg_telnetd_send_response(output, "Incorrect number of parameters.");
	}
	else
	{
		if (strcmp(argv[0], "rmdir") != 0)
			return false;
		getcwd(shell_buf, shell_buf_size);
		dir = opendir(shell_buf);
		if (dir == 0)
		{
			sprintf(shell_buf, "opendir() return -1 %d: %s.", errno, strerror(errno));
			cyg_telnetd_send_response(output, shell_buf);
		}
		else
		{
			retval = closedir(dir);
			if (-1 == retval)
			{
				sprintf(shell_buf, "closedir() return -1 %d: %s.", errno, strerror(errno));
				cyg_telnetd_send_response(output, shell_buf);
			}
			else
			{
				if (strlen(shell_buf) == 0)
					strcpy(shell_buf, "/disk0");
				strncat(shell_buf, "/", 1);
				strncat(shell_buf, argv[1], shell_buf_size);

				retval = rmdir(shell_buf);
				if (ENOTEMPTY == retval)
					cyg_telnetd_send_response(output, "Directory not empty.");
				else if (0 != retval)
				{
					sprintf(shell_buf, "rmdir() return -1 %d: %s.", errno, strerror(errno));
					cyg_telnetd_send_response(output, shell_buf);
				}
				else
					cyg_telnetd_send_response(output, "");
			}
		}
	}
	retval = cyg_fs_setinfo("/disk0", FS_INFO_SYNC, NULL, NULL);
	return true;
}

cyg_bool cyg_sh_rm_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
	Cyg_ErrNo 		retval;
	struct dirent 	dir_data;
	struct dirent	*dir_result;
	DIR 			*dir;
	struct 			stat st;

	if (argc != 2)
	{
		cyg_telnetd_send_response(output, "Incorrect number of parameters.");
	}
	else
	{
		if (strcmp(argv[0], "rm") != 0)
			return false;
		getcwd(shell_buf, shell_buf_size);
		dir = opendir(shell_buf);
		if (dir == 0)
		{
			sprintf(shell_buf, "opendir() return -1 %d: %s.", errno, strerror(errno));
			cyg_telnetd_send_response(output, shell_buf);
		}
		else
		{
			retval = closedir(dir);
			if (-1 == retval)
			{
				sprintf(shell_buf, "closedir() return -1 %d: %s.", errno, strerror(errno));
				cyg_telnetd_send_response(output, shell_buf);
			}
			else
			{
				if (strlen(shell_buf) == 0)
					strcpy(shell_buf, "/disk0");
				strncat(shell_buf, "/", 1);
				strncat(shell_buf, argv[1], shell_buf_size);
				retval = unlink(shell_buf);
				if (EBUSY == retval)
					cyg_telnetd_send_response(output, "File busy.");
				else if (ENOENT == retval)
					cyg_telnetd_send_response(output, "File does not exist.");
				else if (0 != retval)
				{
					sprintf(shell_buf, "rm() return -1 %d: %s.", errno, strerror(errno));
					cyg_telnetd_send_response(output, shell_buf);
				}
				else
					cyg_telnetd_send_response(output, "");
			}
		}
	}
	retval = cyg_fs_setinfo("/disk0", FS_INFO_SYNC, NULL, NULL);
	return true;
}

cyg_bool cyg_sh_mv_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
	Cyg_ErrNo 		retval;
	struct dirent 	dir_data;
	struct dirent	*dir_result;
	DIR 			*dir;
	struct 			stat st;

	if (argc != 3)
	{
		cyg_telnetd_send_response(output, "Incorrect number of parameters.");
	}
	else
	{
		if (strcmp(argv[0], "mv") != 0)
			return false;

		getcwd(shell_buf, shell_buf_size);
		if (strlen(shell_buf) == 0)
			strcpy(shell_buf, "/disk0");
		dir = opendir(shell_buf);
		if (dir == 0)
		{
			sprintf(shell_buf, "opendir() return -1 %d: %s.", errno, strerror(errno));
			cyg_telnetd_send_response(output, shell_buf);
		}
		else
		{
			retval = closedir(dir);
			if (-1 == retval)
			{
				sprintf(shell_buf, "closedir() return -1 %d: %s.", errno, strerror(errno));
				cyg_telnetd_send_response(output, shell_buf);
			}
			else
			{
				if (access(argv[1], F_OK) != 0)
				{
					sprintf(shell_buf, "File does not exist %s", argv[1]);
					cyg_telnetd_send_response(output, shell_buf);
				}
				else if (access(argv[2], F_OK) != 0)
				{
				    if (rename(argv[1], argv[2]) != 0)
				    {
						sprintf(shell_buf, "rename() return -1 %d: %s.", errno, strerror(errno));
						cyg_telnetd_send_response(output, shell_buf);
				    }
				}
				else
				{
					dir = opendir(argv[2]);
					if (-1 != retval)
						sprintf(shell_buf, "%s/%s", argv[2], argv[1]);
					else
						sprintf(shell_buf, "%s", argv[2]);
					closedir(dir);
					if (access(shell_buf, F_OK) != 0)
					{
						if (rename(argv[1], shell_buf) != 0)
						{
							sprintf(shell_buf, "rename() return -1 %d: %s.", errno, strerror(errno));
							cyg_telnetd_send_response(output, shell_buf);
						}
					}
					else
					{
						sprintf(shell_buf, "File already exists %s", argv[2]);
						cyg_telnetd_send_response(output, "File already exists");
					}
				}
			}
			retval = cyg_fs_setinfo("/disk0", FS_INFO_SYNC, NULL, NULL);
			cyg_telnetd_send_response(output, "");
		}
	}
	return true;
}

// From http://orion.math.iastate.edu/burkardt/c_src/hexdump/hexdump.c

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define ERROR 0
#define MAX_INPUT_LINE 80
#define SUCCESS 1

/******************************************************************************/

cyg_bool cyg_sh_hd_handler(FILE *out, FILE *in, FILE *err, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
/******************************************************************************/

/*
  Purpose:

    HEXDUMP is a simple hexadecimal dump program.

  Usage:

    hexdump FILE_NAME > OUTPUT  for command-line use;
    hexdump                     for interactive use.

  Sample output:

    HEXDUMP: Hexadecimal dump of file: box.3ds.

       Address                   Hexadecimal values                  Printable
    --------------  -----------------------------------------------  ----------------

        0 00000000  4d 4d 53 02 00 00 02 00 0a 00 00 00 03 00 00 00  MMS.............
       16 00000010  3d 3d 68 01 00 00 3e 3d 0a 00 00 00 03 00 00 00  ==h...>=........
       32 00000020  00 01 0a 00 00 00 00 00 80 3f 00 40 4e 01 00 00  .........?.@N...
       48 00000030  42 6f 78 30 31 00 00 41 42 01 00 00 10 41 68 00  Box01..AB....Ah.
       64 00000040  00 00 08 00 26 76 0a c2 1f 0c a8 c1 00 00 00 00  ....&v..........
       ...

  Note:

    HEXDUMP is similar to the UNIX octal dump program OD with the "-h" switch,
    but automatically includes a listing of printable ASCII characters.

  Reference:

    Howard Burdick,
    Digital Imaging, Theory and Applications,
    McGraw Hill, 1997, page 219.

  Modified:

    09 August 2001
*/
  long int       addr;
  unsigned char *b;
  unsigned char  buf[20];
  long int       cnt;
  long int       cnt2;
  FILE          *filein;
  char           filein_name[MAX_INPUT_LINE];
  FILE          *fileout;
  char           fileout_name[MAX_INPUT_LINE];
  char           input[MAX_INPUT_LINE];
  long           m;
  long           n;
/*
  Check for the right number of arguments.
*/
  if ( argc < 2 ) {

	sprintf(shell_buf,  "" );
	cyg_telnetd_send_response(out, shell_buf);
	sprintf(shell_buf,  "HEXDUMP:" );
	cyg_telnetd_send_response(out, shell_buf);
	sprintf(shell_buf, "  Please enter the INPUT file name:" );
	cyg_telnetd_send_response(out, shell_buf);

	fgets ( input, MAX_INPUT_LINE, in );

    if ( fgets ( input, MAX_INPUT_LINE, in ) != NULL ) {
      sscanf ( input, "%s", filein_name );
    }
    else {
    	sprintf(shell_buf, "" );
    	cyg_telnetd_send_response(out, shell_buf);
    	sprintf(shell_buf, "HEXDUMP - Fatal error!" );
    	cyg_telnetd_send_response(out, shell_buf);
    	sprintf(shell_buf, "  Input error!" );
    	cyg_telnetd_send_response(out, shell_buf);
      return ERROR;
    }

    sprintf(shell_buf, "" );
	cyg_telnetd_send_response(out, shell_buf);
    sprintf(shell_buf, "HEXDUMP:" );
	cyg_telnetd_send_response(out, shell_buf);
    sprintf(shell_buf, "  Please enter the OUTPUT file name:" );
	cyg_telnetd_send_response(out, shell_buf);

    if ( fgets ( input, MAX_INPUT_LINE, in ) != NULL ) {
      sscanf ( input, "%s", fileout_name );
    }
    else {
    	sprintf(shell_buf, "" );
    	cyg_telnetd_send_response(out, shell_buf);
    	sprintf(shell_buf, "HEXDUMP - Fatal error!" );
    	cyg_telnetd_send_response(out, shell_buf);
    	sprintf(shell_buf, "  Input error!" );
    	cyg_telnetd_send_response(out, shell_buf);
      return 1;
    }

    fileout = fopen ( fileout_name, "w" );

    if ( fileout == NULL ) {
    	sprintf(shell_buf, "" );
    	cyg_telnetd_send_response(out, shell_buf);
    	sprintf(shell_buf, "HEXDUMP - Fatal error!" );
    	cyg_telnetd_send_response(out, shell_buf);
    	sprintf(shell_buf, "  Cannot open the output file %s.", fileout_name );
    	cyg_telnetd_send_response(out, shell_buf);
      return 1;
    }

  }
  else {
    strcpy ( filein_name, argv[1] );
    fileout = out;
  }
/*
  Open the input file.
*/
  filein = fopen ( filein_name, "rb" );

  if ( filein == NULL ) {
	sprintf(shell_buf, "" );
	cyg_telnetd_send_response(out, shell_buf);
    sprintf(shell_buf, "HEXDUMP - Fatal error!" );
	cyg_telnetd_send_response(out, shell_buf);
    sprintf(shell_buf, "  Cannot open the input file %s.", filein_name );
	cyg_telnetd_send_response(out, shell_buf);
    return 1;
  }
/*
  Dump the file contents.
*/
  fprintf ( fileout, "\n" );
  fprintf ( fileout, "HEXDUMP: Hexadecimal dump of file: %s.\n", filein_name );
  fprintf ( fileout, "\n" );
  fprintf ( fileout, "   Address                   Hexadecimal values                  Printable\n" );
  fprintf ( fileout, "--------------  -----------------------------------------------  ----------------\n" );
  fprintf ( fileout, "\n" );

  addr = 0;

  while ( ( cnt = ( long )
    fread ( buf, sizeof ( unsigned char ), 16, filein ) ) > 0 ) {

    b = buf;
/*
  Print the address in decimal and hexadecimal.
*/
    fprintf ( fileout, "%5d %08lx  ", (int) addr, addr );
    addr = addr + 16;
/*
  Print 16 data items, in pairs, in hexadecimal.
*/
    cnt2 = 0;
    for ( m = 0; m < 16; m++ ) {
      cnt2 = cnt2 + 1;
      if ( cnt2 <= cnt ) {
        fprintf ( fileout, "%02x", *b++ );
      }
      else {
        fprintf ( fileout, "  " );
      }
      fprintf ( fileout, " " );
    }
/*
  Print the printable characters, or a period if unprintable.
*/
    fprintf ( fileout, " " );
    cnt2 = 0;
    for ( n = 0; n < 16; n++ ) {
      cnt2 = cnt2 + 1;
      if ( cnt2 <= cnt ) {
        if ( ( buf[n] < 32 ) || ( buf[n] > 126 ) ) {
          fprintf ( fileout, "%c", '.' );
        }
        else {
          fprintf ( fileout, "%c", buf[n] );
        }
      }
    }
    fprintf( fileout, "\n" );
  }

  fclose ( filein );

  fprintf ( fileout, "\n" );
  fprintf ( fileout, "HEXDUMP:\n" );
  fprintf ( fileout, "  End of hexadecimal dump.\n" );

  if (fileout != out)
    fclose ( fileout );

  sprintf(shell_buf, "" );
  sprintf(shell_buf, "HEXDUMP:" );
  sprintf(shell_buf, "  Normal end of execution." );

  return 1;
}

cyg_bool cyg_sh_heap_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
	Cyg_ErrNo 		retval;
	struct mallinfo 	info;

	if (argc != 1)
	{
		cyg_telnetd_send_response(output, "Incorrect number of parameters.");
	}
	else
	{
		if (strcmp(argv[0], "heap") != 0)
			return false;

  		info = mallinfo();
  		sprintf(shell_buf, "arena\t\t%d", info.arena);		cyg_telnetd_send_response(output, shell_buf);
  		sprintf(shell_buf, "fordblks\t%d", info.fordblks);	cyg_telnetd_send_response(output, shell_buf);
  		sprintf(shell_buf, "fsmblks\t\t%d", info.fsmblks);	cyg_telnetd_send_response(output, shell_buf);
  		sprintf(shell_buf, "hblkhd\t\t%d", info.hblkhd);	cyg_telnetd_send_response(output, shell_buf);
  		sprintf(shell_buf, "hblks\t\t%d", info.hblks);		cyg_telnetd_send_response(output, shell_buf);
  		sprintf(shell_buf, "keepcost\t%d", info.keepcost);	cyg_telnetd_send_response(output, shell_buf);
  		sprintf(shell_buf, "maxfree\t\t%d", info.maxfree);	cyg_telnetd_send_response(output, shell_buf);
  		sprintf(shell_buf, "ordblks\t\t%d", info.ordblks);	cyg_telnetd_send_response(output, shell_buf);
  		sprintf(shell_buf, "smblks\t\t%d", info.smblks);	cyg_telnetd_send_response(output, shell_buf);
  		sprintf(shell_buf, "uordblks\t%d", info.uordblks);	cyg_telnetd_send_response(output, shell_buf);
  		sprintf(shell_buf, "usmblks\t\t%d", info.usmblks);	cyg_telnetd_send_response(output, shell_buf);
	}
	return true;
}

cyg_bool cyg_sh_task_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
	Cyg_ErrNo 		retval;
	cyg_handle_t 	thread = 0;
	cyg_uint16 		thread_id = 0;
	cyg_thread_info	thread_info;
	char 		    state[20];

	if (argc != 1)
	{
		cyg_telnetd_send_response(output, "Incorrect number of parameters.");
	}
	else
	{
		if (strcmp(argv[0], "task") != 0)
			return false;

  	  sprintf(shell_buf, "thread\tpri\taddr\t\tstack base\tsize\tused\tpercent\tstate            name");
  	  cyg_telnetd_send_response(output, shell_buf);

			while(cyg_thread_get_next(&thread, &thread_id))
			{
				if (!cyg_thread_get_info(thread, thread_id, &thread_info))
					break;
				switch (thread_info.state)
				{
				case 0 :
					strcpy(state, "running          ");
					break;
				case 1 :
					strcpy(state, "sleeping         ");
					break;
				case 2 :
					strcpy(state, "counted sleep    ");
					break;
				case 3 :
					strcpy(state, "sleep set        ");
					break;
				case 4 :
					strcpy(state, "suspended        ");
					break;
				case 8 :
					strcpy(state, "creating         ");
					break;
				case 16 :
					strcpy(state, "exited           ");
					break;
				default :
					strcpy(state, "                 ");
				}
				sprintf(shell_buf, "%d\t%d\t0x%x\t0x%x\t%d\t%d\t%4.1f\t%s%s",
						thread_info.id,
						thread_info.set_pri,
						thread,
						thread_info.stack_base,
						thread_info.stack_size,
						thread_info.stack_used,
						100.0 * (float)thread_info.stack_used / (float)thread_info.stack_size,
						state,
						thread_info.name
						);
				cyg_telnetd_send_response(output, shell_buf);
			}
	}
	return true;
}

#ifdef CYGPKG_SH_COMMAND_LUA

#ifdef CYGPKG_TELNETD_COMMAND_DLMALLOC

static void *cyg_sh_l_alloc (void *ud, void *ptr, size_t osize, size_t nsize) {
  (void)ud; (void)osize;  /* not used */
  Cyg_Mempool_dlmalloc *mempool = (Cyg_Mempool_dlmalloc *)ud;
  if (nsize == 0) {
    mempool->free((cyg_uint8*)ptr);
    return NULL;
  }
  else
  {
	  // Taken from malloc.cxx
	    cyg_int32 oldsize;

	    // if pointer is NULL, we must malloc it
	    if ( ptr == NULL ) {
	        ptr = mempool->alloc( nsize );
	        return ptr;
	    } // if

	    // if size is 0, we must free it
	    if (nsize == 0) {
	        mempool->free((cyg_uint8*)ptr);
	        return NULL;
	    } // if

	    void *newptr;

	    // otherwise try to resize allocation
	    newptr = mempool->resize_alloc( (cyg_uint8 *)ptr, nsize, &oldsize );

	    if ( NULL == newptr ) {
	        // if resize_alloc doesn't return a pointer, it failed, so we
	        // just have to allocate new space instead, and later copy it

	        CYG_ASSERT( oldsize != 0,
	                    "resize_alloc() couldn't determine allocation size!" );

	        newptr = mempool->alloc( nsize );

	        if ( NULL != newptr ) {
	            memcpy( newptr, ptr, nsize < (size_t) oldsize ? nsize
	                    : (size_t) oldsize );
	            mempool->free((cyg_uint8*) ptr );
	        }
	    }

	    return newptr;
  }
}

static int cyg_sh_panic (lua_State *L) {
  luai_writestringerror(L, "PANIC: unprotected error in call to Lua API (%s)\n",
                   lua_tostring(L, -1));
  return 0;  /* return to Lua to abort */
}

__externC void finalreport (lua_State *L, int status);

lua_State *cyg_sh_luaL_newstate (void *udata) {
  lua_State *L = lua_newstate(cyg_sh_l_alloc, udata);
  if (L) lua_atpanic(L, &cyg_sh_panic);
  return L;
}

static void cyg_sh_l_message (lua_State *L, const char *pname, const char *msg) {
  if (pname) luai_writestringerror(L, "%s: ", pname);
  luai_writestringerror( L, "%s\n", msg);
}

__externC int pmain (lua_State *L);

int cyg_sh_lua_main (FILE *output, FILE *input, FILE *error, int argc, char **argv, void *udata) {
  int status, result;
  lua_State *L = cyg_sh_luaL_newstate(udata);  /* create state */
  L->io_output = output;
  L->io_input = input;
  L->io_error = error;
  if (L == NULL) {
	  cyg_sh_l_message(L, argv[0], "cannot create state: not enough memory");
    return EXIT_FAILURE;
  }
  /* call 'pmain' in protected mode */
  lua_pushcfunction(L, &pmain);
  lua_pushinteger(L, argc);  /* 1st argument */
  lua_pushlightuserdata(L, argv); /* 2nd argument */
  status = lua_pcall(L, 2, 1, 0);
  result = lua_toboolean(L, -1);  /* get result */
  finalreport(L, status);
  lua_close(L);
  return (result && status == LUA_OK) ? EXIT_SUCCESS : EXIT_FAILURE;
}

#endif

__externC int lua_main (FILE *output, FILE *input, FILE *error, int argc, char **argv);

cyg_bool cyg_sh_lua_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
	char **new_argv = (char**)malloc(argc+1 * sizeof(char*));
	int i;
	for (i = 0; i < argc; i++)
		new_argv[i] = argv[i];
	new_argv[argc] = 0;
#ifdef CYGPKG_TELNETD_COMMAND_DLMALLOC
	cyg_sh_lua_main (output, input, error, argc, new_argv, user_data);
#else
	lua_main (output, input, error, argc, new_argv);
#endif
	free(new_argv);
	return true;
}

#endif

// This code came from monitor.c in the httpd and is put at the end
// because these includes muck up other commands related to the file system.

#define _KERNEL
#include <sys/param.h>
#undef _KERNEL
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/errno.h>
#include <sys/time.h>
#include <netdb.h>
#define _KERNEL

#include <sys/sysctl.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip_icmp.h>
#include <net/route.h>
#include <net/if_dl.h>

#include <sys/protosw.h>
#include <netinet/in_pcb.h>
#include <netinet/udp.h>
#include <netinet/tcp.h>
#include <netinet/tcp_timer.h>
#include <netinet/ip_var.h>
#include <netinet/icmp_var.h>
#include <netinet/udp_var.h>
#include <netinet/tcp_var.h>
#ifdef CYGPKG_NET_INET6
#include <netinet/ip6.h>
#include <net/if_var.h>
#include <netinet6/ip6_var.h>
#include <netinet6/in6_var.h>
#include <netinet/icmp6.h>
#endif

#include <sys/mbuf.h>

#include <cyg/io/eth/eth_drv_stats.h>

cyg_bool cyg_sh_network_handler(FILE *output, FILE *input, FILE *error, char *command,
                              int argc, char *argv[], void *arg, void *user_data)
{
    struct ifaddrs *iflist, *ifp;
    char addr[64];
    int i;

	if (argc != 1)
	{
		cyg_telnetd_send_response(output, "Incorrect number of parameters.");
	}
	else
	{

		if(getifaddrs(&iflist)!=0)
			return true;

	    ifp = iflist;
		while( ifp != (struct ifaddrs *)NULL)
		 {
			 if (ifp->ifa_addr->sa_family != AF_LINK)
			 {
				sprintf( shell_buf, "%s", ifp->ifa_name);
				cyg_telnetd_send_response(output, shell_buf);

				sprintf( shell_buf, "\tFlags" );

				for( i = 0; i < 16; i++ )
				{
				 switch( ifp->ifa_flags & (1<<i) )
				   {
				   default: break;
				   case IFF_UP: strcat( shell_buf, " UP" ); break;
				   case IFF_BROADCAST: strcat( shell_buf, " BROADCAST" ); break;
				   case IFF_DEBUG: strcat( shell_buf, " DEBUG" ); break;
				   case IFF_LOOPBACK: strcat( shell_buf, " LOOPBACK" ); break;
				   case IFF_PROMISC: strcat( shell_buf, " PROMISCUOUS" ); break;
				   case IFF_RUNNING: strcat( shell_buf, " RUNNING" ); break;
				   case IFF_SIMPLEX: strcat( shell_buf, " SIMPLEX" ); break;
				   case IFF_MULTICAST: strcat( shell_buf, " MULTICAST" ); break;
				   }
				}
				cyg_telnetd_send_response(output, shell_buf);

				getnameinfo(ifp->ifa_addr, sizeof(*ifp->ifa_addr),
						 addr, sizeof(addr), NULL, 0, NI_NUMERICHOST);
				sprintf( shell_buf, "\tAddress %s", addr);
				cyg_telnetd_send_response(output, shell_buf);

				if (ifp->ifa_netmask)
				{
				getnameinfo(ifp->ifa_netmask, sizeof(*ifp->ifa_netmask),
						   addr, sizeof(addr), NULL, 0, NI_NUMERICHOST);
				sprintf( shell_buf, "\tMask %s", addr);
				cyg_telnetd_send_response(output, shell_buf);
				}

				if (ifp->ifa_broadaddr)
				{
				getnameinfo(ifp->ifa_broadaddr, sizeof(*ifp->ifa_broadaddr),
						   addr, sizeof(addr), NULL, 0, NI_NUMERICHOST);
				sprintf( shell_buf, "\tBroadcast\t%s", addr);
				cyg_telnetd_send_response(output, shell_buf);
				}
			 }

			 ifp = ifp->ifa_next;
		 }

        sprintf(shell_buf, "Protocols" );
		cyg_telnetd_send_response(output, shell_buf);

        sprintf( shell_buf, "\tIPv4" );
    	cyg_telnetd_send_response(output, shell_buf);

		sprintf( shell_buf, "\t\t%s:", "Received" );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t\t%s\t%ld", "Total       ",
				 ipstat.ips_total );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t\t%s\t%ld", "Bad         ",
				 ipstat.ips_badsum+
				 ipstat.ips_tooshort+
				 ipstat.ips_toosmall+
				 ipstat.ips_badhlen+
				 ipstat.ips_badlen+
				 ipstat.ips_noproto+
				 ipstat.ips_toolong
			);
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t\t%s\t%ld", "Reassembled ",
				 ipstat.ips_reassembled );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t\t%s\t%ld", "Delivered   ",
				 ipstat.ips_delivered );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t%s:", "Sent" );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t\t%s\t%ld", "Total       ",
				 ipstat.ips_localout );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t\t%s\t%ld", "Raw         ",
				 ipstat.ips_rawout );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t\t%s\t%ld", "Fragmented  ",
				 ipstat.ips_fragmented );
    	cyg_telnetd_send_response(output, shell_buf);

#ifdef CYGPKG_NET_INET6
        sprintf( shell_buf, "\tIPv6" );
    	cyg_telnetd_send_response(output, shell_buf);

        sprintf( shell_buf, "\t\t%s:", "Received    " );
    	cyg_telnetd_send_response(output, shell_buf);
        sprintf( shell_buf, "\t\t\t%s\t%lld", "Total       ",
				 ip6stat.ip6s_total );
    	cyg_telnetd_send_response(output, shell_buf);
        sprintf( shell_buf, "\t\t\t%s\t%lld\n", "Bad         ",
				 ip6stat.ip6s_tooshort+
				 ip6stat.ip6s_toosmall
			);
    	cyg_telnetd_send_response(output, shell_buf);
        sprintf( shell_buf, "\t\t\t%s\t%lld", "Reassembled ",
				 ip6stat.ip6s_reassembled );
    	cyg_telnetd_send_response(output, shell_buf);
        sprintf( shell_buf, "\t\t\t%s\t%lld", "Delivered   ",
				 ip6stat.ip6s_delivered );

    	cyg_telnetd_send_response(output, shell_buf);
        sprintf( shell_buf, "\t\t%s:", "Sent" );
    	cyg_telnetd_send_response(output, shell_buf);
        sprintf( shell_buf, "\t\t\t%s\t%lld", "Total       ",
				 ip6stat.ip6s_localout );
    	cyg_telnetd_send_response(output, shell_buf);
        sprintf( shell_buf, "\t\t\t%s\t%lld", "Raw         ",
				 ip6stat.ip6s_rawout );
    	cyg_telnetd_send_response(output, shell_buf);
        sprintf( shell_buf, "\t\t\t%s\t%lld", "Fragmented  ",
				 ip6stat.ip6s_fragmented );
    	cyg_telnetd_send_response(output, shell_buf);
#endif

    	sprintf( shell_buf, "\tICMPv4" );
    	cyg_telnetd_send_response(output, shell_buf);

    	sprintf( shell_buf, "\t\t%s:", "Received    " );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "ECHO       ",
				 icmpstat.icps_inhist[ICMP_ECHO] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "ECHO REPLY  ",
				 icmpstat.icps_inhist[ICMP_ECHOREPLY] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld\n", "UNREACH     ",
				 icmpstat.icps_inhist[ICMP_UNREACH] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "REDIRECT    ",
				 icmpstat.icps_inhist[ICMP_REDIRECT] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Other       ",
				 icmpstat.icps_inhist[ICMP_SOURCEQUENCH]+
				 icmpstat.icps_inhist[ICMP_ROUTERADVERT]+
				 icmpstat.icps_inhist[ICMP_ROUTERSOLICIT]+
				 icmpstat.icps_inhist[ICMP_TIMXCEED]+
				 icmpstat.icps_inhist[ICMP_PARAMPROB]+
				 icmpstat.icps_inhist[ICMP_TSTAMP]+
				 icmpstat.icps_inhist[ICMP_TSTAMPREPLY]+
				 icmpstat.icps_inhist[ICMP_IREQ]+
				 icmpstat.icps_inhist[ICMP_IREQREPLY]+
				 icmpstat.icps_inhist[ICMP_MASKREQ]+
				 icmpstat.icps_inhist[ICMP_MASKREPLY]
			);
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Bad         ",
				 icmpstat.icps_badcode+
				 icmpstat.icps_tooshort+
				 icmpstat.icps_checksum+
				 icmpstat.icps_badlen+
				 icmpstat.icps_bmcastecho
			);

    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t%s:", "Sent" );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "ECHO        ",
				 icmpstat.icps_outhist[ICMP_ECHO] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "ECHO REPLY ",
				 icmpstat.icps_outhist[ICMP_ECHOREPLY] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "UNREACH     ",
				 icmpstat.icps_outhist[ICMP_UNREACH] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "REDIRECT    ",
				 icmpstat.icps_outhist[ICMP_REDIRECT] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Other       ",
				 icmpstat.icps_inhist[ICMP_SOURCEQUENCH]+
				 icmpstat.icps_outhist[ICMP_ROUTERADVERT]+
				 icmpstat.icps_outhist[ICMP_ROUTERSOLICIT]+
				 icmpstat.icps_outhist[ICMP_TIMXCEED]+
				 icmpstat.icps_outhist[ICMP_PARAMPROB]+
				 icmpstat.icps_outhist[ICMP_TSTAMP]+
				 icmpstat.icps_outhist[ICMP_TSTAMPREPLY]+
				 icmpstat.icps_outhist[ICMP_IREQ]+
				 icmpstat.icps_outhist[ICMP_IREQREPLY]+
				 icmpstat.icps_outhist[ICMP_MASKREQ]+
				 icmpstat.icps_outhist[ICMP_MASKREPLY]
			);
    	cyg_telnetd_send_response(output, shell_buf);

#ifdef CYGPKG_NET_INET6
    	sprintf( shell_buf, "\tICMPv6" );
    	cyg_telnetd_send_response(output, shell_buf);

    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t%s:", "Received" );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%lld", "ECHO        ",
				 icmp6stat.icp6s_inhist[ICMP_ECHO] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%lld", "ECHO REPLY  ",
				 icmp6stat.icp6s_inhist[ICMP_ECHOREPLY] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%lld", "UNREACH     ",
				 icmp6stat.icp6s_inhist[ICMP_UNREACH] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%lld", "REDIRECT    ",
				 icmp6stat.icp6s_inhist[ICMP_REDIRECT] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%lld", "Other       ",
				 icmp6stat.icp6s_inhist[ICMP_SOURCEQUENCH]+
				 icmp6stat.icp6s_inhist[ICMP_ROUTERADVERT]+
				 icmp6stat.icp6s_inhist[ICMP_ROUTERSOLICIT]+
				 icmp6stat.icp6s_inhist[ICMP_TIMXCEED]+
				 icmp6stat.icp6s_inhist[ICMP_PARAMPROB]+
				 icmp6stat.icp6s_inhist[ICMP_TSTAMP]+
				 icmp6stat.icp6s_inhist[ICMP_TSTAMPREPLY]+
				 icmp6stat.icp6s_inhist[ICMP_IREQ]+
				 icmp6stat.icp6s_inhist[ICMP_IREQREPLY]+
				 icmp6stat.icp6s_inhist[ICMP_MASKREQ]+
				 icmp6stat.icp6s_inhist[ICMP_MASKREPLY]
			);
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%lld", "Bad         ",
				 icmp6stat.icp6s_badcode+
				 icmp6stat.icp6s_tooshort+
				 icmp6stat.icp6s_checksum+
				 icmp6stat.icp6s_badlen
			);
    	cyg_telnetd_send_response(output, shell_buf);

    	sprintf( shell_buf, "\t\t%s:", "Sent" );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%lld", "ECHO        ",
				 icmp6stat.icp6s_outhist[ICMP_ECHO] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%lld", "ECHO REPLY  ",
				 icmp6stat.icp6s_outhist[ICMP_ECHOREPLY] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%lld", "UNREACH     ",
				 icmp6stat.icp6s_outhist[ICMP_UNREACH] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%lld</tr>\n", "REDIRECT    ",
				 icmp6stat.icp6s_outhist[ICMP_REDIRECT] );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%lld", "Other       ",
				 icmp6stat.icp6s_inhist[ICMP_SOURCEQUENCH]+
				 icmp6stat.icp6s_outhist[ICMP_ROUTERADVERT]+
				 icmp6stat.icp6s_outhist[ICMP_ROUTERSOLICIT]+
				 icmp6stat.icp6s_outhist[ICMP_TIMXCEED]+
				 icmp6stat.icp6s_outhist[ICMP_PARAMPROB]+
				 icmp6stat.icp6s_outhist[ICMP_TSTAMP]+
				 icmp6stat.icp6s_outhist[ICMP_TSTAMPREPLY]+
				 icmp6stat.icp6s_outhist[ICMP_IREQ]+
				 icmp6stat.icp6s_outhist[ICMP_IREQREPLY]+
				 icmp6stat.icp6s_outhist[ICMP_MASKREQ]+
				 icmp6stat.icp6s_outhist[ICMP_MASKREPLY]
			);
    	cyg_telnetd_send_response(output, shell_buf);

#endif

    	sprintf( shell_buf, "\tUDP" );
    	cyg_telnetd_send_response(output, shell_buf);

    	sprintf( shell_buf, "\t\t%s:", "Received" );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Total       ",
				 udpstat.udps_ipackets );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Bad         ",
				 udpstat.udps_hdrops+
				 udpstat.udps_badsum+
				 udpstat.udps_badlen+
				 udpstat.udps_noport+
				 udpstat.udps_noportbcast+
				 udpstat.udps_fullsock
			);
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t%s:", "Sent" );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Total      ",
				 udpstat.udps_opackets );
    	cyg_telnetd_send_response(output, shell_buf);

    	sprintf( shell_buf, "\tTCP" );
    	cyg_telnetd_send_response(output, shell_buf);

    	sprintf( shell_buf, "\t\t%s:", "Connections" );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Initiated   ",
				 tcpstat.tcps_connattempt );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Accepted    ",
				 tcpstat.tcps_accepts );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Established",
				 tcpstat.tcps_connects );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Closed      ",
				 tcpstat.tcps_closed );
    	cyg_telnetd_send_response(output, shell_buf);

    	sprintf( shell_buf, "\t\t%s:", "Received" );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Packets     ",
				 tcpstat.tcps_rcvtotal );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Data Packets",
				 tcpstat.tcps_rcvpack );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Bytes       ",
				 tcpstat.tcps_rcvbyte );
    	cyg_telnetd_send_response(output, shell_buf);

    	sprintf( shell_buf, "\t\t%s:", "Sent" );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld\t", "Packets    ",
				 tcpstat.tcps_sndtotal );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Data Packets",
				 tcpstat.tcps_sndpack );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t\t%s\t%ld", "Bytes       ",
				 tcpstat.tcps_sndbyte );
    	cyg_telnetd_send_response(output, shell_buf);

        sprintf(shell_buf, "Mbufs" );
    	cyg_telnetd_send_response(output, shell_buf);

        sprintf( shell_buf, "\tSummary");
    	cyg_telnetd_send_response(output, shell_buf);

    	sprintf( shell_buf, "\t\t%s\t%ld", "Mbufs       ",
				 mbstat.m_mbufs );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t%s\t%ld", "Clusters    ",
				 mbstat.m_clusters );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t%s\t%ld", "Free Clusters",
				 mbstat.m_clfree );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t%s\t%ld", "Drops       ",
				 mbstat.m_drops );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t%s\t%ld", "Waits       ",
				 mbstat.m_wait );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t%s\t%ld", "Drains      ",
				 mbstat.m_drain );
    	cyg_telnetd_send_response(output, shell_buf);
#if defined(CYGPKG_NET_FREEBSD_STACK)
    	sprintf( shell_buf, "\t\t%s\t%ld", "Copy Fails  ",
				 mbstat.m_mcfail );
    	cyg_telnetd_send_response(output, shell_buf);
    	sprintf( shell_buf, "\t\t%s\t%ld", "Pullup Fails",
				 mbstat.m_mpfail );
    	cyg_telnetd_send_response(output, shell_buf);
#endif

		u_long *mtypes;
#if defined(CYGPKG_NET_FREEBSD_STACK)
		mtypes = mbtypes;
#else
		mtypes = mbstat.m_mtypes;
#endif

        sprintf( shell_buf, "\tTypes" );
    	cyg_telnetd_send_response(output, shell_buf);

		sprintf( shell_buf, "\t\t%s\t%ld", "FREE      ",
				 mtypes[MT_FREE] );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t%s\t%ld", "DATA        ",
				 mtypes[MT_DATA] );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t%s\t%ld", "HEADER      ",
				 mtypes[MT_HEADER] );
    	cyg_telnetd_send_response(output, shell_buf);
#if !defined(CYGPKG_NET_FREEBSD_STACK)
		sprintf( shell_buf, "\t\t%s\t%ld", "SOCKET      ",
				 mtypes[MT_SOCKET] );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t%s\t%ld", "PCB         ",
				 mtypes[MT_PCB] );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t%s\t%ld", "RTABLE      ",
				 mtypes[MT_RTABLE] );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t%s\t%ld", "HTABLE      ",
				 mtypes[MT_HTABLE] );
    	cyg_telnetd_send_response(output, shell_buf);
		sprintf( shell_buf, "\t\t%s\t%ld", "ATABLE      ",
				 mtypes[MT_ATABLE] );
    	cyg_telnetd_send_response(output, shell_buf);
#endif
		sprintf( shell_buf, "\t\t%s\t%ld", "SONAME      ",
				 mtypes[MT_SONAME] );
    	cyg_telnetd_send_response(output, shell_buf);
#if !defined(CYGPKG_NET_FREEBSD_STACK)
		sprintf( shell_buf, "\t\t%s\t%ld", "SOOPTS      ",
				 mtypes[MT_SOOPTS] );
    	cyg_telnetd_send_response(output, shell_buf);
#endif
		sprintf( shell_buf, "\t\t%s\t%ld", "FTABLE      ",
				 mtypes[MT_FTABLE] );
    	cyg_telnetd_send_response(output, shell_buf);

	}
	return true;
}

// Don't put new commands after here unless they are using the network.

/* ----------------------------------------------------------------- */
/* end of sh.c                                                       */
