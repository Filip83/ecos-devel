#ifndef __BLOCK__
#define __BLOCK__
//=============================================================================
//
//      block.h
//
//      SIMPLEFS block manipulation layer
//
//=============================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2004 Free Software Foundation, Inc.                        
//
// eCos is free software; you can redistribute it and/or modify it under    
// the terms of the GNU General Public License as published by the Free     
// Software Foundation; either version 2 or (at your option) any later      
// version.                                                                 
//
// eCos is distributed in the hope that it will be useful, but WITHOUT      
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or    
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License    
// for more details.                                                        
//
// You should have received a copy of the GNU General Public License        
// along with eCos; if not, write to the Free Software Foundation, Inc.,    
// 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.            
//
// As a special exception, if other files instantiate templates or use      
// macros or inline functions from this file, or you compile this file      
// and link it with other works to produce a work based on this file,       
// this file does not by itself cause the resulting work to be covered by   
// the GNU General Public License. However the source code for this file    
// must still be made available in accordance with section (3) of the GNU   
// General Public License v2.                                               
//
// This exception does not invalidate any other reasons why a work based    
// on this file might be covered by the GNU General Public License.         
// -------------------------------------------                              
// ####ECOSGPLCOPYRIGHTEND####                                              
//=============================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):     Filip Adamec <filip.adamec.ez2@gmail.com>
// Contributors:  
// Date:          2013-10-22
// Purpose:       
// Description:   
//              
//
//####DESCRIPTIONEND####
//
//=============================================================================

typedef unsigned int block_id_t;
typedef unsigned short block_address_t;
typedef unsigned short block_len_t;

int block_write_super(fbfs_super_block *super);

int block_write(fbfs_super_block *super, block_id_t block, block_address_t offset,
                block_len_t len, const void *data, cyg_uint32 *writed);
int block_rewrite(block_id_t block, block_address_t offset, block_len_t len, 
					 const void *data);
int block_read(fbfs_super_block *super, block_id_t block, block_address_t offset, 
                block_len_t len, void *data, cyg_uint32 *readed);
int block_errase(fbfs_super_block *super,block_id_t block);


int block_alloc(fbfs_super_block *super, block_id_t *new_block);
int block_alloc_reserved(fbfs_super_block *super, block_id_t *new_block);
int block_free(fbfs_super_block *super, block_id_t block_id);
int block_write_bitmap(fbfs_super_block *super);
int block_read_bitmap(fbfs_super_block *super);

int block_set_used(fbfs_super_block *super, block_id_t block, cyg_bool save);
int block_set_free(fbfs_super_block *super, block_id_t block, cyg_bool save);

#endif