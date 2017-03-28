#ifndef __NODE__
#define __NODE__
//=============================================================================
//
//      node.h
//
//      SIMPLEFS node manipulation layer
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

typedef cyg_uint32 node_id_t;


fbfs_inode *node_alloc(fbfs_super_block *super,mode_t mode );
int node_free(fbfs_super_block *super, fbfs_inode * node);

int node_read(fbfs_super_block *super,fbfs_inode *node,node_id_t node_id);
fbfs_inode *node_alloc_read(fbfs_super_block *super,node_id_t node_id);
int node_write(fbfs_super_block *super, fbfs_inode *node);

int node_read_data(fbfs_super_block *super, fbfs_inode *node, cyg_uint32 pos,
			void *data, cyg_uint32 *len);
int node_write_data(fbfs_super_block *super, fbfs_inode *node, cyg_uint32 pos,  
			const void *data, cyg_uint32 *len);

int node_alloc_space(fbfs_super_block *super,fbfs_inode *node, size_t size);
int node_free_space(fbfs_super_block *super,fbfs_inode *node);
int node_write_bitmap(fbfs_super_block *super, node_id_t node_id);
int node_read_bitmap(fbfs_super_block *super);

int node_set_free(fbfs_super_block *super, node_id_t node_id,cyg_bool save);
int node_set_used(fbfs_super_block *super, node_id_t node_id,cyg_bool save);
#endif /* __NODE__ */
