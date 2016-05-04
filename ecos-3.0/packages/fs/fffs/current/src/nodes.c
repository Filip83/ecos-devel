/*
 * nodes.c
 *
 * Created: 25.9.2012 15:47:38
 *  Author: Filip
 */
#ifdef CYGPKG_KERNEL
#include <cyg/kernel/kapi.h>
#endif
#include <cyg/io/spi.h>
#include <cyg/io/spi_avr32.h>
#include <cyg/io/dataflash.h>
#include <cyg/io/flash.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/cyg_trac.h>
#include <cyg/infra/diag.h>
#include <cyg/crc/crc.h>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <cyg/fs/nodes.h>


//#define FFFS_DIAG_OUT(message) printf("%s",message)



/**
* Funkce seète dvì adresy tak že pokud adresa zasahuje za konec
* pamìti flash dojde k jeji upravì tak že ukazuje na zaèátek.
* Funkce v podstatì vytváøí kruhoé adresování.
* @param info - informaèni struktura
* @param opa  - prvni adresa
* @param opb  - druhá adresa
* @return     - souèet obou adres upraveny.
*/
cyg_uint32 add_address(t_flash_info *info, cyg_uint32 opa, cyg_uint32 opb)
{
	cyg_uint32 addr;
	addr = opa + opb;
	addr -= (addr >= info->flash_size) ? (info->flash_size) : 0;
	return addr ;
}

/**
* Funkce odeète dvì adresy tak že pokud adresa zasahuje za zaèátek
* pamìti flash dojde k jeji upravì tak že ukazuje na zaèátek.
* Funkce v podstatì vytváøí kruhoé adresování.
* @param info - informaèni struktura
* @param opa  - prvni adresa
* @param opb  - druhá adresa
* @return     - souèet obou adres upraveny.
*/
cyg_uint32 sub_address(t_flash_info *info, cyg_uint32 opa, cyg_uint32 opb)
{
	cyg_int32 addr;
	addr = opa - opb;
	addr += (addr < 0) ? (info->flash_size /*- 1*/) : 0;
	return addr;
}

cyg_uint32 fffs_write(t_flash_info *info, cyg_uint32 fffs_addr,const void *data, cyg_uint32 length)
{
	cyg_uint32 ret;
	cyg_flashaddr_t err_addr;
	const cyg_uint8 *ram = (cyg_uint8*)data;

	if((info->used_space + length) < info->flash_size)
	{

		if((fffs_addr + length) <= info->flash_size)
		{
			if((ret = cyg_flash_program((info->flash.start + fffs_addr),data,length,&err_addr))
				!= CYG_FLASH_ERR_OK)
			{
				return ret;
			}
		}
		else
		{
			cyg_uint32 len = info->flash_size - fffs_addr/* - 1*/;
			if((ret = fffs_write(info,fffs_addr,data,len)) != len)
				return ret;

			if((ret = fffs_write(info,0 ,ram + len,length - len))
				!= (length - len))
				return ret;
		}
		return length;
	}

	return FFFS_ERR_FULL;
}

cyg_uint32 fffs_read(t_flash_info *info, cyg_uint32 fffs_addr, void *data, cyg_uint32 length)
{
	cyg_uint32 ret;
	cyg_flashaddr_t err_addr;
	cyg_uint8 *ram = (cyg_uint8*)data;


	if((fffs_addr + length) <= info->flash_size)
	{
		if((ret = cyg_flash_read((info->flash.start + fffs_addr),data,length,&err_addr))
			!= CYG_FLASH_ERR_OK)
		{
			return ret;
		}
	}
	else
	{
		cyg_uint32 len = info->flash_size - fffs_addr/* - 1*/;
		if((ret = fffs_read(info,fffs_addr,data,len)) != len)
			return ret;

		if((ret = fffs_read(info,0 ,ram + len,length - len))
			!= (length - len))
			return ret;
	}
	return length;

}

cyg_uint32 fffs_move(t_flash_info *info, cyg_uint32 dst_addr, cyg_uint32 src_addr, cyg_uint32 len)
{
	//prvni se musi zapsat noda a pak data
	cyg_uint32 ret;
	cyg_uint32  length;
	cyg_uint8 *cpy_buff;

	if(len < FFFS_CPY_BUFF_LIMIT )
	{
		length = len;
		cpy_buff = (cyg_uint8 *)malloc(len);
	}
	else
	{
		length = FFFS_CPY_BUFF_LIMIT;
		cpy_buff = (cyg_uint8 *)malloc(FFFS_CPY_BUFF_LIMIT);
	}

	if(cpy_buff == NULL)
	{
		free(cpy_buff);
		return FFFS_ERR_RAM_LIMIT;
	}

	do
	{
		if((ret = fffs_read(info,src_addr,cpy_buff,length)) != length)
		{
			free(cpy_buff);
			return ret;
		}

		if((ret = fffs_write(info,dst_addr,cpy_buff,length)) != length)
		{
			free(cpy_buff);
			return ret;
		}

		src_addr = add_address(info,src_addr,length);
		dst_addr = add_address(info,dst_addr,length);
		len -= length;
		if(len < FFFS_CPY_BUFF_LIMIT)
			length = len;

	} while (len);

	free(cpy_buff);
	return FFFS_ERR_OK;
}

cyg_uint32 fffs_move_node(t_flash_info *info, t_node_info *nfile)
{
	cyg_uint32 ret;
	t_node_info new_file;
	char name[FFFS_MAX_NAME_SIZE];
	if(nfile->node.mode & FFFS_MODE_NAMED)
	{
		if((ret = fffs_get_name(info,nfile,name)) != FFFS_ERR_OK)
			return ret;

		if((ret = fffs_create_named_node(name,info,&new_file,nfile->node.mode))
			!= FFFS_ERR_OK)
			return ret;

	}
	else
	{
		if((ret = fffs_create_node(info,&new_file,nfile->node.mode))
			!= FFFS_ERR_OK)
			return ret;
	}

	if((ret = fffs_move(info,new_file.node.data_addr,nfile->node.data_addr,
			nfile->node.node_length)) != FFFS_ERR_OK)
			return ret;

	new_file.node.node_length = nfile->node.node_length;

	if((ret = fffs_close_node(info,&new_file)) != FFFS_ERR_OK)
		return ret;
	memcpy(nfile,&new_file,sizeof(t_node_info));
	return FFFS_ERR_OK;
}


cyg_uint32 fffs_delete(t_flash_info *info, t_node_info *nfile)
{
	cyg_uint32 length;
	nfile->node.node_type = NODE_TYPE_DELTED;

	if(nfile->node.mode & FFFS_MODE_NAMED)
		length = sizeof(t_node) + FFFS_MAX_NAME_SIZE;
	else
		length = sizeof(t_node);

	nfile->node.crc16 =  cyg_crc16((unsigned char*)&nfile->node,
		sizeof(t_node) - sizeof(cyg_uint16));

	if(fffs_write(info,sub_address(info,nfile->node.data_addr, length),nfile,sizeof(t_node))
		!= sizeof(t_node))
		return FFFS_ERR_ADDR;

	if(nfile->node.mode & FFFS_MODE_SYSTEM)
		info->system_space -= (nfile->node.node_length + length);
	return FFFS_ERR_OK;
}

cyg_uint32 fffs_errase(t_flash_info *info, cyg_uint32 start, cyg_uint32 end)
{
	cyg_flashaddr_t err_addr;
	cyg_int32 tmp;

	tmp = end - start;
	tmp *= (tmp < 0) ? (-1) : 1;
	tmp--;

	if((cyg_uint32)tmp > info->flash_size)
		return FFFS_ERR_ADDR;

	//info->fsm = errase_errasing;

	start += info->flash.start;
	end   += info->flash.start;

	if(end >= start)
	{
		do
		{
			cyg_flash_erase(start, 1, &err_addr);
			start += info->flash.block_info->block_size;
			info->progres++;
		}while(start < end);
	}
	else
	{
		do
		{
			cyg_flash_erase(start, 1, &err_addr);
			start += info->flash.block_info->block_size;
			info->progres++;
		}while(start < info->flash.end);

		start = info->flash.start;

		do
		{
			cyg_flash_erase(start, 1, &err_addr);
			start += info->flash.block_info->block_size;
			info->progres++;
		}while(start < end);
	}
	//info->fsm = errase_doen;
	return FFFS_ERR_OK;
}

cyg_uint32 fffs_get_name(t_flash_info *info,t_node_info *nfile, char *name)
{
	if(nfile->node.mode & FFFS_MODE_NAMED)
	{
		cyg_uint32 name_addr = sub_address(info,nfile->node.data_addr,FFFS_MAX_NAME_SIZE);
		if(fffs_read(info,name_addr,name,FFFS_MAX_NAME_SIZE) != FFFS_MAX_NAME_SIZE)
			return FFFS_ERR_ADDR;
		return FFFS_ERR_OK;
	}
	name[0] = 0;
	return FFFS_ERR_NO_NAMED_NODE;
}

cyg_uint32 fffs_find_start_node(t_flash_info *info)
{
	cyg_uint32 i;
	int node_cnt = 0;
	cyg_uint32 ret ;
	cyg_uint16 crc;
	t_node tmp_node;

	CYG_ASSERT(info,"No flash info");


	for(i = 0; i < info->flash_size; i += info->flash.block_info->block_size)
	{
		if((ret = fffs_read(info,i,&tmp_node,sizeof(t_node)))
			!= sizeof(t_node))
			return ret;

		if(tmp_node.node_type == NODE_TYPE_START)
		{
			node_cnt++;
			info->start_node_addr = i;
			crc = cyg_crc16((unsigned char*)&tmp_node,
				sizeof(t_node) - sizeof(cyg_uint16));

			if(crc != tmp_node.crc16)
				return FFFS_ERR_CRC;
		}
	}

	if(node_cnt == 0)
		return FFFS_ERR_START_NODE;
	if(node_cnt == 1)
		return FFFS_ERR_OK;

	return FFFS_ERR_NEED_CHECK;
}

cyg_uint32 fffs_repair(t_flash_info *info, cyg_uint32 corrupted_node_addr)
{
	cyg_uint32 ret;
	cyg_uint8  *buffer;
	cyg_uint32 block, errase_to;

	buffer = (cyg_uint8*)malloc(info->flash.block_info->block_size);
	if(buffer == NULL)
		return FFFS_ERR_RAM_LIMIT;

	block = (corrupted_node_addr/info->flash.block_info->block_size)*
		info->flash.block_info->block_size;

	if((ret = fffs_read(info,block,buffer,info->flash.block_info->block_size))
		!= info->flash.block_info->block_size)
	{
		free(buffer);
		return ret;
	}

	memset(buffer + (corrupted_node_addr - block),-1,
		info->flash.block_info->block_size - (corrupted_node_addr - block));

	if((ret = fffs_write(info,block,buffer,info->flash.block_info->block_size))
		!= info->flash.block_info->block_size)
	{
		free(buffer);
		return ret;
	}

	free(buffer);

	block = add_address(info,block,info->flash.block_info->block_size);
	errase_to = sub_address(info,info->start_node_addr,info->flash.block_info->block_size);
	return fffs_errase(info, block, errase_to);

}




cyg_uint32 fffs_init(t_flash_info *info)
{
	int files_cnt = 0;
	t_node tmp_node,tmp_node2;
	cyg_uint32 ret;
	cyg_uint16 crc;

	CYG_ASSERT(info,"No flash info");

	memset(info,0,sizeof(t_flash_info));

	info->last_error = FFFS_ERR_OK;

	if(cyg_flash_init(NULL) != CYG_FLASH_ERR_OK)
	{
		CYG_ASSERT(false,"Flash initialization filed");
		return FFFS_ERR_DETECT;
	}

	if(cyg_flash_get_info(0,&info->flash) != CYG_FLASH_ERR_OK)
		return FFFS_ERR_DETECT;

	info->n_flash_chips = FFFS_FLASH_CHIPS;
	info->flash_size = info->flash.block_info->block_size*
	    info->flash.block_info->blocks*info->n_flash_chips;

	/*if(info->flash_errase_state != errase_doen)
		return FFFS_ERR_NEED_CHECK;*/
#ifdef CYGPKG_KERNEL
	cyg_mutex_init(&info->write_mutex);
#endif
	ret = fffs_find_start_node(info);

	if(ret == FFFS_ERR_START_NODE)
	{
		return FFFS_ERR_NEAD_FORMAT;
		//return fffs_format(info, false);
	}

	if(ret == FFFS_ERR_NEED_CHECK)
	{
		return FFFS_ERR_NEED_CHECK;
	}

	if(ret != FFFS_ERR_OK)
		return ret;

	if((ret = fffs_read(info,info->start_node_addr,&tmp_node,sizeof(t_node)))
		!= sizeof(t_node))
		return ret;

	crc = cyg_crc16((unsigned char*)&tmp_node,
		sizeof(t_node) - sizeof(cyg_uint16));

	if(crc != tmp_node.crc16)
	{
		//return fffs_format(info, true);
		return FFFS_ERR_NEAD_FORMAT;
	}

	if(tmp_node.node_type != NODE_TYPE_START)
	{
		return FFFS_ERR_START_NODE;
	}

	tmp_node2 = tmp_node;
	do
	{
		info->cur_node = tmp_node;
		files_cnt++;
		tmp_node2 = tmp_node;

		if((ret = fffs_read(info,tmp_node.next_node,&tmp_node,sizeof(t_node)))
			!= sizeof(t_node))
			return ret;

		crc = cyg_crc16((unsigned char*)&tmp_node,
			sizeof(t_node) - sizeof(cyg_uint16));

		if(crc != tmp_node.crc16 && tmp_node.node_type != NODE_TYPE_END)
		{
			if((ret = fffs_repair(info,tmp_node2.next_node))
				!= FFFS_ERR_OK)
				return ret;
			tmp_node.node_type = NODE_TYPE_END;
			break;
			//return FFFS_ERR_CRC;
		}

		if(((tmp_node.node_type == NODE_TYPE_RECORD) ||
		   (tmp_node.node_type == NODE_TYPE_SYSTEM)) &&
		   (tmp_node.mode & FFFS_MODE_NAMED)         &&
		   (info->num_named_nodes < FFFS_NAMED_NODE_MAX) )
		   {
			   info->named_nodes[info->num_named_nodes].node = tmp_node;

			   if((ret = fffs_read(info,sub_address(info,tmp_node.data_addr,FFFS_MAX_NAME_SIZE),
				   info->named_nodes[info->num_named_nodes++].name,FFFS_MAX_NAME_SIZE))
					!= FFFS_MAX_NAME_SIZE)
					return ret;

			   if(tmp_node.node_type == NODE_TYPE_SYSTEM)
				   info->system_space += tmp_node.node_length
						+ sizeof(t_node) + FFFS_MAX_NAME_SIZE;

		   }
		if(((tmp_node.node_type == NODE_TYPE_RECORD) ||
		   (tmp_node.node_type == NODE_TYPE_SYSTEM)) &&
		   !(tmp_node.mode & FFFS_MODE_NAMED))
		{

		   info->num_numbered++;
		   if(tmp_node.node_type == NODE_TYPE_SYSTEM)
				   info->system_space += tmp_node.node_length
						+ sizeof(t_node);
		}

	} while (tmp_node.node_type != NODE_TYPE_END);


	if(info->cur_node.mode & FFFS_MODE_NAMED)
		info->end_node_addr = sub_address(info,info->cur_node.data_addr,
			sizeof(t_node) + FFFS_MAX_NAME_SIZE);
	else
		info->end_node_addr = sub_address(info,info->cur_node.data_addr,
		sizeof(t_node));

	if(info->cur_node.next_node == NODE_TYPE_DELTED)
	{
		if((ret = fffs_read(info,info->end_node_addr,&tmp_node,sizeof(t_node)))
			!= sizeof(t_node))
			return ret;

		crc = cyg_crc16((unsigned char*)&tmp_node,
			sizeof(t_node) - sizeof(cyg_uint16));

		if(crc != tmp_node.crc16)
			return FFFS_ERR_CRC;

		if(info->cur_node.mode & FFFS_MODE_NAMED)
			info->end_node_addr = sub_address(info,info->cur_node.data_addr,
				sizeof(t_node) + FFFS_MAX_NAME_SIZE);
		else
			info->end_node_addr = sub_address(info,info->cur_node.data_addr,
			sizeof(t_node));
	}

	info->num_files     = files_cnt - 1;
	info->used_space    = sub_address(info,info->cur_node.next_node, info->start_node_addr);
	return FFFS_ERR_OK ;
}

cyg_uint32 fffs_get_current_node(t_flash_info *info, t_node_info *nfile)
{
	CYG_ASSERT(info,"No flash info");
	CYG_ASSERT(nfile,"No flash file info");

	nfile->node         = info->cur_node;
	nfile->file_cur_pos = 0;
	nfile->name_node    = false;
	nfile->modified     = false;
	return FFFS_ERR_OK ;
}

cyg_uint32 fffs_set_current_node(t_flash_info *info, t_node_info *nfile)
{
	CYG_ASSERT(info,"No flash info");
	CYG_ASSERT(nfile,"No flash node info");

	info->cur_node = nfile->node;
	return FFFS_ERR_OK;
}

cyg_uint32 fffs_get_next_node(t_flash_info *info, t_node_info *nfile)
{
	t_node tmp_node;
	cyg_uint32 ret;
	cyg_uint16 crc;

	CYG_ASSERT(info,"No flash info");

	if(nfile == NULL)
	{
		if((ret = fffs_read(info,info->cur_node.next_node,&tmp_node,sizeof(t_node)))
			!= sizeof(t_node))
			return ret;
	}
	else
	{
		if((ret = fffs_read(info,nfile->node.next_node,&tmp_node,sizeof(t_node)))
			!= sizeof(t_node))
			return ret;
	}

	if(tmp_node.node_type == NODE_TYPE_END)
	{
		return FFFS_ERR_END_NODE;
	}

	crc = cyg_crc16((unsigned char*)&tmp_node,
		sizeof(t_node) - sizeof(cyg_uint16));

	if(crc != tmp_node.crc16)
		return FFFS_ERR_CRC;

	if(tmp_node.node_type == NODE_TYPE_RECORD ||
	   tmp_node.node_type == NODE_TYPE_SYSTEM ||
	   tmp_node.node_type == NODE_TYPE_DELTED)
	{

		info->cur_node = tmp_node;
		if(nfile != NULL)
			return fffs_get_current_node(info,nfile);
		else
			return FFFS_ERR_OK;
	}

	return FFFS_ERR_CORUPTED;
}

cyg_uint32 fffs_get_prev_node(t_flash_info *info, t_node_info *nfile)
{
	t_node tmp_node;
	cyg_uint32 ret;
	cyg_uint16 crc;

	CYG_ASSERT(info,"No flash info");

	if(nfile == NULL)
	{
		if((ret = fffs_read(info,info->cur_node.prev_node,&tmp_node,sizeof(t_node)))
			!= sizeof(t_node))
			return ret;
	}
	else
	{
		if((ret = fffs_read(info,nfile->node.prev_node,&tmp_node,sizeof(t_node)))
			!= sizeof(t_node))
			return ret;
	}

	crc = cyg_crc16((unsigned char*)&tmp_node,
		sizeof(t_node) - sizeof(cyg_uint16));

	if(crc != tmp_node.crc16)
		return FFFS_ERR_CRC;

	if(tmp_node.node_type == NODE_TYPE_START)
	{
		return FFFS_ERR_START_NODE;
	}

	info->cur_node = tmp_node;
	if(nfile != NULL)
		return fffs_get_current_node(info,nfile);
	else
		return FFFS_ERR_OK;
}



cyg_uint32 fffs_get_first_node(t_flash_info *info, t_node_info *nfile)
{
	t_node tmp_node;
	cyg_uint32 ret;
	cyg_uint16 crc;

	CYG_ASSERT(info,"No flash info");

	if((ret = fffs_read(info,info->start_node_addr,&tmp_node,sizeof(t_node)))
		!= sizeof(t_node))
		return ret;

	crc = cyg_crc16((unsigned char*)&tmp_node,
		sizeof(t_node) - sizeof(cyg_uint16));

	if(crc != tmp_node.crc16)
		return FFFS_ERR_CRC;

	info->cur_node = tmp_node;
	if(nfile != NULL)
		return fffs_get_current_node(info,nfile);
	else
		return FFFS_ERR_OK;

}

cyg_uint32 fffs_get_last_node(t_flash_info *info, t_node_info *nfile)
{
	t_node tmp_node;
	cyg_uint32 ret;
	cyg_uint16 crc;

	CYG_ASSERT(info,"No flash info");

	if((ret = fffs_read(info,info->end_node_addr,&tmp_node,sizeof(t_node)))
		!= sizeof(t_node))
		return ret;

	crc = cyg_crc16((unsigned char*)&tmp_node,
		sizeof(t_node) - sizeof(cyg_uint16));

	if(crc != tmp_node.crc16 && tmp_node.node_type != NODE_TYPE_FREE)
		return FFFS_ERR_CRC;

	info->cur_node = tmp_node;
	if(nfile != NULL)
		return fffs_get_current_node(info,nfile);
	else
		return FFFS_ERR_OK;
}


cyg_uint32 fffs_open_node(t_flash_info *info,t_node_info *nfile, cyg_uint32 node_id)
{
	t_node_info tmp_node;
	cyg_uint32 ret;
	cyg_uint16 crc;

	CYG_ASSERT(info,"No flash info");

	if(info->cur_node.node_id == node_id)
		return FFFS_ERR_OK;

	if(node_id > info->num_files || node_id == 0)
		return FFFS_ERR_BAD_NODE_ID;

	if(node_id > (info->num_files/2))
	{
		if((ret = fffs_get_last_node(info,&tmp_node)) != FFFS_ERR_OK)
			return ret;

		while(tmp_node.node.node_id != node_id)
		{
			if((ret = fffs_read(info,tmp_node.node.prev_node,&tmp_node.node,sizeof(t_node)))
					!= sizeof(t_node))
					return ret;

			crc = cyg_crc16((unsigned char*)&tmp_node.node,
				sizeof(t_node) - sizeof(cyg_uint16));

			if(crc != tmp_node.node.crc16)
				return FFFS_ERR_CRC;

			if(tmp_node.node.node_type == NODE_TYPE_START)
				return FFFS_ERR_START_NODE;

			if(tmp_node.node.node_type == NODE_TYPE_END)
				return FFFS_ERR_END_NODE;

		}

	}
	else
	{
		if((ret = fffs_get_first_node(info,&tmp_node)) != FFFS_ERR_OK)
			return ret;

		while(tmp_node.node.node_id != node_id)
		{
			if((ret = fffs_read(info,tmp_node.node.next_node,&tmp_node.node,sizeof(t_node)))
					!= sizeof(t_node))
					return ret;

			crc = cyg_crc16((unsigned char*)&tmp_node.node,
				sizeof(t_node) - sizeof(cyg_uint16));

			if(crc != tmp_node.node.crc16)
				return FFFS_ERR_CRC;

			if(tmp_node.node.node_type == NODE_TYPE_START)
				return FFFS_ERR_START_NODE;

			if(tmp_node.node.node_type == NODE_TYPE_END)
				return FFFS_ERR_END_NODE;

		}
	}

	info->cur_node = tmp_node.node;

	if(nfile != NULL)
		return fffs_get_current_node(info,nfile);
	else
		return FFFS_ERR_OK;
}

#ifdef FFFS_ENABLE_APPEND
cyg_uint32 fffs_open_node_append(t_flash_info *info, t_node_info *nfile)
{
	cyg_uint32 ret;
	t_node_info new_file;
	CYG_ASSERT(info,"No flash info");
	CYG_ASSERT(nfile,"No flash file info");


	if((ret = fffs_create_node(info,&new_file,nfile->node.mode))
		!= FFFS_ERR_OK)
		return ret;

	if((ret = fffs_move(info,new_file.node.data_addr,nfile->node.data_addr,nfile->node.node_length))
		!= FFFS_ERR_OK)
		return ret;

	new_file.node.node_length = nfile->node.node_length;

	if((ret = fffs_delete(info,nfile)) != FFFS_ERR_OK)
		return ret;

	*nfile = new_file;

	return FFFS_ERR_NAMED_NOT_FOUND;
}
#endif

cyg_uint32 fffs_read_node(t_flash_info *info,t_node_info *nfile, void *data, cyg_uint32 length)
{
	cyg_uint32 ret;
	CYG_ASSERT(info,"No flash info");
	CYG_ASSERT(nfile,"No flash file nfile");

	if(nfile->file_cur_pos < nfile->node.node_length)
	{
		if((nfile->file_cur_pos + length) >= nfile->node.node_length)
		   length = nfile->node.node_length - nfile->file_cur_pos;

		if((ret = fffs_read(info,add_address(info,nfile->node.data_addr, nfile->file_cur_pos)
			,data,length))
				!= length)
				return ret;

		nfile->file_cur_pos += length;
		return length;
	}
	return FFFS_ERR_OK;
}

cyg_uint32 fffs_write_node(t_flash_info *info, t_node_info *nfile, const void *data, cyg_uint32 length)
{
    cyg_uint32 ret;

	CYG_ASSERT(info,"No flash info");
	CYG_ASSERT(nfile,"No flash file nfile");
	CYG_ASSERT(data,"No flash file nfile");
#ifdef FFFS_ENABLE_REWRITE
	if((nfile->node.node_type == NODE_TYPE_RECORD) ||
		(nfile->node.node_type == NODE_TYPE_SYSTEM) &&
		(nfile->file_cur_pos < nfile->node.node_length))
	{
		if((nfile->file_cur_pos + length) >= nfile->node.node_length)
			length = nfile->node.node_length - nfile->file_cur_pos;
	}
	else
#endif
		if(nfile->node.node_type != NODE_TYPE_NEW)
			return FFFS_ERR_CANT_WRITE;

	if((ret = fffs_write(info,add_address(info,nfile->node.data_addr, nfile->file_cur_pos)
			,data,length)) != length)
			return ret;

	nfile->file_cur_pos     += length;
	if(nfile->node.node_type == NODE_TYPE_NEW)
		nfile->node.node_length += length;
	return length;
}

cyg_uint32 fffs_close_node(t_flash_info *info, t_node_info *nfile)
{
	cyg_uint32 ret;
	cyg_uint32 length, node_addr;

    CYG_ASSERT(info,"No flash info");
    CYG_ASSERT(nfile,"No flash file info");

	if(nfile->node.node_type != NODE_TYPE_NEW)
		return FFFS_ERR_OK;

	if(nfile->node.mode & FFFS_MODE_SYSTEM)
		nfile->node.node_type = NODE_TYPE_SYSTEM;
	else
		nfile->node.node_type = NODE_TYPE_RECORD;

	if(nfile->node.mode & FFFS_MODE_NAMED)
		length = sizeof(t_node) + FFFS_MAX_NAME_SIZE;
	else
		length = sizeof(t_node);

	nfile->node.next_node = add_address(info,nfile->node.data_addr,
		nfile->node.node_length);

	node_addr = sub_address(info,nfile->node.data_addr,
		length);


	nfile->node.crc16 = cyg_crc16((unsigned char*)&nfile->node,
		sizeof(t_node) - sizeof(cyg_uint16));
	if((ret = fffs_write(info,node_addr
			,&nfile->node,sizeof(t_node))) != sizeof(t_node))
			return ret;

	if(nfile->name_node)
		info->named_nodes[nfile->named_id].node = nfile->node;
	info->end_node_addr   = node_addr;
	info->used_space     += nfile->node.node_length + length;
	if(nfile->node.mode & FFFS_MODE_SYSTEM)
		info->system_space += nfile->node.node_length + length;
	info->num_files++;
#ifdef CYGPKG_KERNEL
	cyg_mutex_unlock(&info->write_mutex);
#endif
	return FFFS_ERR_OK;
}

cyg_uint32 fffs_create_node(t_flash_info *info, t_node_info *nfile, cyg_uint16 mode)
{
	cyg_uint32 ret;
	CYG_ASSERT(info,"No flash info");
	CYG_ASSERT(nfile,"No flash file info");

	if((info->flash_size - info->used_space) <
		(info->system_space + 2*info->flash.block_info->block_size))
		return FFFS_ERR_FULL;

	if((ret = fffs_get_last_node(info, nfile)) != FFFS_ERR_OK)
		return ret;

	nfile->file_cur_pos     = 0;
	nfile->name_node        = false;
	nfile->node.create_time = time (NULL);
	nfile->node.node_id     = (mode == FFFS_MODE_RECORD) ?  ++info->num_numbered : -1;
	if(info->cur_node.mode & FFFS_MODE_NAMED)
		nfile->node.prev_node   = sub_address(info,info->cur_node.data_addr,
		sizeof(t_node) + FFFS_MAX_NAME_SIZE);
	else
		nfile->node.prev_node   = sub_address(info,info->cur_node.data_addr, sizeof(t_node));
	nfile->node.node_length = 0;
	nfile->node.node_type   = NODE_TYPE_NEW;
	nfile->node.next_node   = 0;
	nfile->node.data_addr   = add_address(info,info->cur_node.next_node, sizeof(t_node));

	nfile->node.mode        = mode;
	nfile->name_node        = false;
#ifdef CYGPKG_KERNEL
	cyg_mutex_lock(&info->write_mutex);
#endif
	return FFFS_ERR_OK;
}




cyg_uint32 fffs_open_named_node(const char *file_name, t_flash_info *info, t_node_info *nfile)
{
	cyg_uint32 i;
	CYG_ASSERT(info,"No flash info");
	CYG_ASSERT(nfile,"No flash file info");

	for(i = 0; i < info->num_named_nodes; i++)
	{
		if(!strcmp(file_name,info->named_nodes[i].name))
		{
			info->cur_node = info->named_nodes[i].node;
			//mozna overit spravnost
			return fffs_get_current_node(info,nfile);
		}
	}
	return FFFS_ERR_NAMED_NOT_FOUND;
}

#ifdef FFFS_ENABLE_APPEND
cyg_uint32 fffs_open_named_node_apped(const char *file_name, t_flash_info *info, t_node_info *nfile)
{
	cyg_uint32 ret;
	cyg_uint32 i;
	CYG_ASSERT(info,"No flash info");
	CYG_ASSERT(nfile,"No flash file info");

	for(i = 0; i < info->num_named_nodes; i++)
	{
		if(!strcmp(file_name,info->named_nodes[i].name))
		{
			info->cur_node = info->named_nodes[i].node;
			//mozna overit spravnost
			return fffs_delete(info,nfile);
			fffs_get_current_node(info,nfile);

			if((ret = fffs_create_named_node(file_name,info,nfile,info->cur_node.mode))
				!= FFFS_ERR_OK)
				return ret;

			if((ret = fffs_move(info,nfile->node.data_addr,info->cur_node.data_addr,info->cur_node.node_length))
				!= FFFS_ERR_OK)
				return ret;

			nfile->node.node_length = info->cur_node.node_length;



		}
	}
	return FFFS_ERR_NAMED_NOT_FOUND;
}
#endif


cyg_uint32 fffs_create_named_node(const char *file_name, t_flash_info *info, t_node_info *nfile, cyg_uint16 mode)
{
	cyg_uint32 ret;
	CYG_ASSERT(info,"No flash info");
	CYG_ASSERT(nfile,"No flash file info");

	cyg_uint8  s_len = strlen(file_name) + 1;
	if(info->num_named_nodes < FFFS_NAMED_NODE_MAX && s_len < FFFS_MAX_NAME_SIZE)
	{
		t_node_info tmp_node;
		if((ret = fffs_create_node(info, &tmp_node,FFFS_MODE_NAMED | mode)) != FFFS_ERR_OK)
			return ret;

		if((ret = fffs_write_node(info,&tmp_node,file_name,s_len))
		   != s_len)
		   return ret;

		*nfile                  = tmp_node;
		nfile->file_cur_pos     = 0;
		nfile->node.node_length = 0;
		nfile->node.data_addr  += FFFS_MAX_NAME_SIZE;
		nfile->name_node        = true;


		info->named_nodes[info->num_named_nodes].node = tmp_node.node;
		nfile->named_id = info->num_named_nodes;
		strcpy(info->named_nodes[info->num_named_nodes++].name,file_name);
		return FFFS_ERR_OK;
	}
	return FFFS_ERR_NAMED_LIMIT;
}

cyg_uint32 fffs_crete_start_node(t_flash_info *info)
{
	cyg_uint32 ret, new_start_addr;
	t_node_info node_info;
	CYG_ASSERT(info,"No flash info");

	node_info.file_cur_pos     = 0;
	node_info.name_node        = false;
	node_info.node.create_time = time (NULL);
	node_info.node.node_id     = 0;
	node_info.node.prev_node   = 0;
	node_info.node.node_length = 0;
	node_info.node.node_type   = NODE_TYPE_START;
	node_info.node.mode        = FFFS_MODE_RECORD;

	info->used_space           = 0;


	if((ret = fffs_get_last_node(info,NULL)) != FFFS_ERR_OK)
	{
		new_start_addr = 0;
	}
	else
	{
		if(info->cur_node.node_type == NODE_TYPE_FREE)
			new_start_addr = 0;
		else
			new_start_addr = (info->cur_node.next_node/info->flash.block_info->block_size + 2)
				*info->flash.block_info->block_size;
	}

	if(new_start_addr >= info->flash_size)
		new_start_addr = 0;

	//printf("New start node at address: 0x%X\n",new_start_addr);

	node_info.node.next_node   = new_start_addr + sizeof(t_node);
	node_info.node.data_addr   = new_start_addr + sizeof(t_node);

	node_info.node.crc16 = cyg_crc16((unsigned char*)&node_info.node,
		sizeof(t_node) - sizeof(cyg_uint16));

	if((ret = fffs_write(info,new_start_addr
			,&node_info.node,sizeof(t_node))) != sizeof(t_node))
			return ret;
	info->used_space += ret;
	return new_start_addr;
}
/*
cyg_uint32 fffs_erase_fsm(t_flash_info *info, cyg_bool complete)
{
	cyg_uint32 ret, i;
	cyg_uint32 errase_block_start,errase_block_end;
	cyg_uint32 copy_to, copy_from, copy_len;
	cyg_uint32 new_start_addr;
	t_flash_info new_info;
	t_node_info  new_node;

	if(!complete)
	{
		memset(&new_info,0,sizeof(t_flash_info));

		switch(info->fsm.flash_fsm)
		{
			case start:
				if((ret = fffs_get_last_node(info,NULL)) != FFFS_ERR_OK)
					return ret;


				new_start_addr = (info->cur_node.next_node/info->flash.block_info->block_size + 2)
					*info->flash.block_info->block_size;
				if(new_start_addr >= info->flash_size)
					new_start_addr = 0;

				errase_block_start = info->start_node_addr;
				errase_block_end   = info->cur_node.next_node;


				copy_len  = errase_block_end - errase_block_start;
				copy_len *= (copy_len < 0) ? (-1):1;

				info->progres_tasks = info->num_files + copy_len/info->flash.block_info->block_size + 2;
				info->progres       = 0;

				new_info.start_node_addr = new_start_addr;
				new_info.end_node_addr   = new_start_addr;
				new_info.flash           = info->flash;
				new_info.flash_size      = info->flash_size;
				new_info.n_flash_chips   = info->n_flash_chips;
				new_info.num_files       = 0;
				new_info.used_space      = 0;
				new_info.num_named_nodes = 0;
				new_info.system_space    = 0;
				//new_info.used_space      = 0;

				copy_to = add_address(info,new_start_addr, sizeof(t_node));

				info->fsm.flash_fsm = crete_new_start;
			break;
			case crete_new_start:
				if(fffs_crete_start_node(info) != new_start_addr)
					return FFFS_ERR_ADDR;

				info->fsm.flash_fsm = move_files;
				break;
			case move_files:


		}
}*/

cyg_uint32 fffs_erase(t_flash_info *info, cyg_bool complete)
{
	cyg_uint32 ret;
	cyg_uint32 errase_block_start,errase_block_end;
	cyg_uint32 copy_len;
	cyg_uint32 new_start_addr;
	t_flash_info new_info;
	t_node_info  new_node;


	if(!complete)
	{
		//info->fsm = start;
		memset(&new_info,0,sizeof(t_flash_info));

		if((ret = fffs_get_last_node(info,NULL)) != FFFS_ERR_OK)
			return ret;


		new_start_addr = (info->cur_node.next_node/info->flash.block_info->block_size + 2)
			*info->flash.block_info->block_size;
		if(new_start_addr >= info->flash_size)
			new_start_addr = 0;

		errase_block_start = info->start_node_addr;
		errase_block_end   = info->cur_node.next_node;


		copy_len  = errase_block_end - errase_block_start;
		copy_len *= (copy_len < 0) ? (-1):1;

		info->progres_tasks = info->num_files + copy_len/info->flash.block_info->block_size + 2;
		info->progres       = 0;

		new_info.start_node_addr = new_start_addr;
		new_info.end_node_addr   = new_start_addr;
		new_info.flash           = info->flash;
		new_info.flash_size      = info->flash_size;
		new_info.n_flash_chips   = info->n_flash_chips;
		new_info.num_files       = 0;
		new_info.used_space      = 0;
		new_info.num_named_nodes = 0;
		new_info.system_space    = 0;
		//new_info.used_space      = 0;

		//copy_to = add_address(info,new_start_addr, sizeof(t_node));



		//info->fsm = cretate_start_node;
		if(fffs_crete_start_node(info) != new_start_addr)
			return FFFS_ERR_ADDR;

		//info->fsm = moving_files;
		if((ret = fffs_get_first_node(info,&new_node)) != FFFS_ERR_OK)
			return ret;

		do
		{
			if(new_node.node.node_type == NODE_TYPE_SYSTEM)
			{

				if((ret = fffs_move_node(&new_info,&new_node)) != FFFS_ERR_OK)
					return ret;

				info->progres++;

				if((ret = fffs_get_current_node(info,&new_node)) != FFFS_ERR_OK)
					return ret;


			}

		} while (fffs_get_next_node(info,&new_node) == FFFS_ERR_OK);

		//info->fsm = moving_files;

		memcpy(info,&new_info,sizeof(t_flash_info));

		info->progres++;


	}
	else
	{
		errase_block_start = 0;
		errase_block_end   = info->flash_size;
	}

	//info->fsm = start_errase;
	ret =  fffs_errase(info,errase_block_start,errase_block_end);
	//info->fsm = done;
	return ret;

}

cyg_uint32 fffs_erasex()
{
	int i = 0;
	cyg_flashaddr_t err_addr;
	cyg_flash_info_t flash;
    cyg_uint32 errase_block_start,errase_block_end;


	if(cyg_flash_get_info(0,&flash) != CYG_FLASH_ERR_OK)
		return FFFS_ERR_DETECT;

	errase_block_start = flash.start;
	errase_block_end   = flash.end;

	do
	{
		cyg_flash_erase(errase_block_start, 1, &err_addr);
		errase_block_start += flash.block_info->block_size;
		i++;
	} while (errase_block_start < errase_block_end);

	return FFFS_ERR_OK;
}

//-----------------------------------------------------------------------------
// Run checkerboard and inverse checkerboard writes over each sector in turn.

cyg_uint32 run_test_1
(t_flash_info *f_info)
{
	int status;
	cyg_uint32 i, j;
	cyg_uint32 errors = 0;
	cyg_flashaddr_t base_addr, err_addr;
	cyg_uint8 *wbuf,*rbuf;

	diag_printf ("Test 1 - write and verify checkerboard and inverse checkerboard.\n");

	wbuf = (cyg_uint8*)malloc(f_info->flash.block_info->block_size);
	if(wbuf == NULL)
		return FFFS_ERR_RAM_LIMIT;

	rbuf = (cyg_uint8*)malloc(f_info->flash.block_info->block_size);
	if(rbuf == NULL)
	{
		free(wbuf);
		return FFFS_ERR_RAM_LIMIT;
	}

	f_info->progres_tasks = f_info->flash.block_info->blocks * 8;
	f_info->progres       = 0;
	// Iterate over all flash sectors.
	for (i = 0; i < f_info->flash.block_info->blocks; i++)
	{
		base_addr =  f_info->flash.start + (i *  f_info->flash.block_info->block_size);

		// Erase block.
		status = cyg_flash_erase (base_addr, 1, &err_addr);
		if (status != FLASH_ERR_OK) {
			diag_printf ("Flash erase error : %s\n", cyg_flash_errmsg (status));
			errors ++;
		}

		f_info->progres ++;
		// Set up buffer with checkerboard.
		for (j = 0; j < BUF_SIZE;) {
			wbuf [j++] = 0x55;
			wbuf [j++] = 0xAA;
		}

		f_info->progres ++;
		// Write the checkerboard to the entire sector.
		for (j = 0; j <  f_info->flash.block_info->block_size; j += BUF_SIZE) {
			status = cyg_flash_program (base_addr + j, wbuf, BUF_SIZE, &err_addr);
			if (status != FLASH_ERR_OK) {
				diag_printf ("Flash write error : %s\n", cyg_flash_errmsg (status));
				errors ++;
			}
		}

		f_info->progres ++;
		// Read back the checkerboard and verify.
		for (j = 0; j <  f_info->flash.block_info->block_size; j += BUF_SIZE) {
			status = cyg_flash_read (base_addr + j, rbuf, BUF_SIZE, &err_addr);
			if (status != FLASH_ERR_OK) {
				diag_printf ("Flash read error : %s\n", cyg_flash_errmsg (status));
				errors ++;
			}
			else if (memcmp (rbuf, wbuf, BUF_SIZE) != 0) {
				diag_printf ("Flash read data corruption (0x%08X - 0x%08X).\n", base_addr + j, base_addr + j + BUF_SIZE - 1);
				errors ++;
			}
		}

		f_info->progres ++;
		// Erase block.
		status = cyg_flash_erase (base_addr, 1, &err_addr);
		if (status != FLASH_ERR_OK) {
			diag_printf ("Flash erase error : %s\n", cyg_flash_errmsg (status));
			errors ++;
		}

		f_info->progres ++;

		// Set up buffer with inverse checkerboard.
		for (j = 0; j < BUF_SIZE;) {
			wbuf [j++] = 0xAA;
			wbuf [j++] = 0x55;
		}

		f_info->progres ++;

		// Write the checkerboard to the entire sector.
		for (j = 0; j <  f_info->flash.block_info->block_size; j += BUF_SIZE) {
			status = cyg_flash_program (base_addr + j, wbuf, BUF_SIZE, &err_addr);
			if (status != FLASH_ERR_OK) {
				diag_printf ("Flash write error : %s\n", cyg_flash_errmsg (status));
				errors ++;
			}
		}

		f_info->progres ++;

		// Read back the checkerboard and verify.
		for (j = 0; j < f_info->flash.block_info->block_size; j += BUF_SIZE) {
			status = cyg_flash_read (base_addr + j, rbuf, BUF_SIZE, &err_addr);
			if (status != FLASH_ERR_OK) {
				diag_printf ("Flash read error : %s\n", cyg_flash_errmsg (status));
				errors ++;
			}
			else if (memcmp (rbuf, wbuf, BUF_SIZE) != 0) {
				diag_printf ("Flash read data corruption (0x%08X - 0x%08X).\n", base_addr + j, base_addr + j + BUF_SIZE - 1);
				errors ++;
			}
		}

		f_info->progres ++;
	}
	free(wbuf);
	free(rbuf);
	return errors;
}

//-----------------------------------------------------------------------------
// Write and verify counting sequence over all device.

cyg_uint32 run_test_2
(t_flash_info *f_info)
{
	int status;
	cyg_uint32 i, j;
	cyg_uint32 errors = 0;
	cyg_flashaddr_t base_addr, err_addr;
	cyg_uint8 *wbuf,*rbuf;

	diag_printf ("Test 2 - write and verify counting sequence.\n");

	wbuf = (cyg_uint8*)malloc(f_info->flash.block_info->block_size);
	if(wbuf == NULL)
		return FFFS_ERR_RAM_LIMIT;

	rbuf = (cyg_uint8*)malloc(f_info->flash.block_info->block_size);
	if(rbuf == NULL)
	{
		free(wbuf);
		return FFFS_ERR_RAM_LIMIT;
	}

	f_info->progres_tasks = f_info->flash.block_info->blocks * 3;
	f_info->progres       = 0;

	// Erase all flash sectors.
	for (i = 0; i <  f_info->flash.block_info->blocks; i++) {
		base_addr =   f_info->flash.start + (i *  f_info->flash.block_info->block_size);

		// Erase block.
		status = cyg_flash_erase (base_addr, 1, &err_addr);
		if (status != FLASH_ERR_OK) {
			diag_printf ("Flash erase error : %s\n", cyg_flash_errmsg (status));
			errors ++;
		}
	}

	f_info->progres ++;
	// Write counting sequence.
	base_addr =   f_info->flash.start;
	for (i = 0; i < ( f_info->flash.end -  f_info->flash.start) / 4;) {
		for (j = 0; j < BUF_SIZE;) {
			wbuf [j++] = (cyg_uint8) ((i >> 0) & 0xFF);
			wbuf [j++] = (cyg_uint8) ((i >> 8) & 0xFF);
			wbuf [j++] = (cyg_uint8) ((i >> 16) & 0xFF);
			wbuf [j++] = (cyg_uint8) ((i >> 24) & 0xFF);
			i++;
		}
		status = cyg_flash_program (base_addr, wbuf, BUF_SIZE, &err_addr);
		if (status != FLASH_ERR_OK) {
			diag_printf ("Flash write error : %s\n", cyg_flash_errmsg (status));
			errors ++;
		}
		base_addr += BUF_SIZE;
	}

	f_info->progres ++;
	// Verify counting sequence.
	base_addr =  f_info->flash.start;
	for (i = 0; i < ( f_info->flash.end -  f_info->flash.start) / 4;) {
		for (j = 0; j < BUF_SIZE;) {
			wbuf [j++] = (cyg_uint8) ((i >> 0) & 0xFF);
			wbuf [j++] = (cyg_uint8) ((i >> 8) & 0xFF);
			wbuf [j++] = (cyg_uint8) ((i >> 16) & 0xFF);
			wbuf [j++] = (cyg_uint8) ((i >> 24) & 0xFF);
			i++;
		}
		status = cyg_flash_read (base_addr, rbuf, BUF_SIZE, &err_addr);
		if (status != FLASH_ERR_OK) {
			diag_printf ("Flash read error : %s\n", cyg_flash_errmsg (status));
			errors ++;
		}
		else if (memcmp (rbuf, wbuf, BUF_SIZE) != 0) {
			diag_printf ("Flash read data corruption (0x%08X - 0x%08X).\n", base_addr, base_addr + BUF_SIZE - 1);
			errors ++;
		}
		base_addr += BUF_SIZE;
		f_info->progres ++;
	}
	free(wbuf);
	free(rbuf);
	return errors;
}

//-----------------------------------------------------------------------------
// Perform non-aligned buffer read/write tests, spanning sector boundaries.

cyg_uint32 run_test_3
(t_flash_info *f_info)
{
	int status;
	cyg_uint32 i, count;
	cyg_uint32 errors = 0;
	cyg_flashaddr_t base_addr, err_addr;
	cyg_uint8 *wbuf,*rbuf;

	diag_printf ("Test 3 - test non-aligned writes and reads.\n");

	wbuf = (cyg_uint8*)malloc(f_info->flash.block_info->block_size);
	if(wbuf == NULL)
		return FFFS_ERR_RAM_LIMIT;

	rbuf = (cyg_uint8*)malloc(f_info->flash.block_info->block_size);
	if(rbuf == NULL)
	{
		free(wbuf);
		return FFFS_ERR_RAM_LIMIT;
	}

	f_info->progres_tasks = 255;
	f_info->progres       = 0;
	// Fill the write buffer with a basic counting sequence.
	count = 0;
	for (i = 0; i < BUF_SIZE;) {
		wbuf [i++] = (cyg_uint8) ((count >> 0) & 0xFF);
		wbuf [i++] = (cyg_uint8) ((count >> 8) & 0xFF);
		count++;
	}

	// Assuming 256 byte pages gives 256 possible alignments.
	base_addr =  f_info->flash.start +  f_info->flash.block_info->block_size;
	for (i = 1; i <= 256; i++) {
		f_info->progres = i;
		// Erase sectors either side of sector boundary.
		status = cyg_flash_erase (base_addr - 1, 1, &err_addr);
		if (status != FLASH_ERR_OK) {
			diag_printf ("Flash erase error : %s\n", cyg_flash_errmsg (status));
			errors ++;
		}
		status = cyg_flash_erase (base_addr, 1, &err_addr);
		if (status != FLASH_ERR_OK) {
			diag_printf ("Flash erase error : %s\n", cyg_flash_errmsg (status));
			errors ++;
		}

		// Write data spanning sector boundary.
		status = cyg_flash_program (base_addr - i, wbuf, BUF_SIZE, &err_addr);
		if (status != FLASH_ERR_OK) {
			diag_printf ("Flash write error : %s\n", cyg_flash_errmsg (status));
			errors ++;
		}

		// Verify data spanning sector boundary.
		status = cyg_flash_read (base_addr - i, rbuf, BUF_SIZE, &err_addr);
		if (status != FLASH_ERR_OK) {
			diag_printf ("Flash read error : %s\n", cyg_flash_errmsg (status));
			errors ++;
		}
		else if (memcmp (rbuf, wbuf, BUF_SIZE) != 0) {
			diag_printf ("Flash read data corruption (0x%08X - 0x%08X).\n", base_addr - i, base_addr - i + BUF_SIZE - 1);
			errors ++;
		}

		// Use next sector boundary for next test.
		base_addr +=  f_info->flash.block_info->block_size;
		if (base_addr >=  f_info->flash.end)
		base_addr =  f_info->flash.start +  f_info->flash.block_info->block_size;
	}
	free(wbuf);
	free(rbuf);
	return errors;
}

cyg_uint32 fffs_test(t_flash_info *info,cyg_uint32 test_id)
{
	if(test_id == 0)
		return run_test_1(info);
	else if(test_id == 1)
		return run_test_2(info);
	else
		return run_test_3(info);

	return FFFS_ERR_OK;
}

cyg_uint32 fffs_format(t_flash_info *info, cyg_bool errase)
{
	cyg_uint32 ret;

	CYG_ASSERT(info,"No flash info");

	if(errase)
		if((ret = fffs_erase(info,errase)) != FFFS_ERR_OK)
			return ret;

   fffs_crete_start_node(info);

	return fffs_init(info);
}

cyg_uint32 fffs_check(t_flash_info *info)
{
	//t_node_info tmp_node;
	//cyg_uint32 ret;


	/*switch(info->flash_errase_state)
	{
	case errase_moving_files:
		return fffs_erase(info,false);
		break;
	case errase_create_start:
	case errase_errasing:
		if((ret = fffs_get_last_node(info,&tmp_node)) != FFFS_ERR_OK)
			return ret;

		return fffs_errase(info,(tmp_node.node.next_node/info->flash.block_info->block_size + 1)
			*info->flash.block_info->block_size,
			info->start_node_addr - 1);
		break;
	case errase_doen:
	default:
		break;

	}*/
	return FFFS_ERR_OK;
}

const char * fffs_print_error(cyg_uint32 error)
{
	if(error > 0)
		return cyg_flash_errmsg(error);

	switch(error)
	{
	case FFFS_ERR_DETECT:
		return "Can not detect flash chip";
	case FFFS_ERR_ADDR:
		return "Flash wrong address error";
	case FFFS_ERR_START_NODE:
		return "Start node problem";
	case FFFS_ERR_END_NODE:
		return "End node problem";
	case FFFS_ERR_WRITE:
		return "Cannot write";
	case FFFS_ERR_NAMED_NOT_FOUND:
		return "Named record not found";
	case FFFS_ERR_NAMED_LIMIT:
		return "Named record limit reached";
	case FFFS_ERR_RAM_LIMIT:
		return "Cannot alloc buffer to move dta in flash";
	case FFFS_ERR_NEED_CHECK:
		return "Filesystem need check";
	case FFFS_ERR_CANT_WRITE:
		return "Cannot write";
	case FFFS_ERR_FULL:
		return "Flash is full";
	case FFFS_ERR_BAD_NODE_ID:
		return "Node id is higher than number of files.";
	case FFFS_ERR_CORUPTED:
		return "File structure corrupeted.";
	case FFFS_ERR_CRC:
		return "CRC error";
	default:
		return "Unkown error";
	}
}

void mexit(int c)
{
	exit(c);
}
