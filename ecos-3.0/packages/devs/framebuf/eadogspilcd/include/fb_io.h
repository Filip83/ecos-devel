/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   kbd_io.h
 * Author: filip
 *
 * Created on 22. listopadu 2016, 19:20
 */

#ifndef KBD_IO_H
#define KBD_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#define CYG_KBD_SET_CONFIG_INTERVAL         0
#define CYG_KBD_SET_CONFIG_ENABLE           1
#define CYG_KBD_SET_CONFIG_DISABLE          2
#define CYG_KBD_SET_CONFIG_CALLBACK         3
#define CYG_KBD_SET_CONFIG_CALLBACK_REMOVE  4
#define CYG_KBD_SET_CONFIG_DEFAULT_INTERVAL 5
    
#define CYG_KBD_GET_CONFIG_INTERVAL         0
#define CYG_KBD_GET_CONFIG_ENABLED          1


#ifdef __cplusplus
}
#endif

#endif /* KBD_IO_H */

