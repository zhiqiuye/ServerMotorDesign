/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                          (c) Copyright 2003-2007; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                           MASTER INCLUDES
*
*                                     ST Microelectronics STM32
*                                              with the
*                                   IAR STM32-SK Evaluation Board
*
* Filename      : includes.h
* Version       : v1.00
* Programmer(s) : FT
*********************************************************************************************************
*/

#ifndef  __INCLUDES_H__
#define  __INCLUDES_H__

#include    <stdio.h>
#include    <string.h>
#include    <ctype.h>
#include    <stdlib.h>
#include    <stdarg.h>

#include    <ucos_ii.h>

#include    <cpu.h>
#include    <lib_def.h>
#include    <lib_mem.h>
#include    <lib_str.h>
#include    <lib_ascii.h>

//#include    <stm32f10x_conf.h>
//#include    <stm32f10x_lib.h>

//#include    <app_cfg.h>
//#include    <bsp.h>

#if (APP_CFG_LCD_EN > 0)
#include    <arm_comm.h>
#include    <drv_glcd_cnfg.h>
#include    <drv_glcd.h>
#include    <glcd_ll.h>
#include    <drv_spi1.h>
#endif

#if (APP_CFG_PROBE_COM_EN == DEF_ENABLED)
#include    <probe_com.h>

#if (APP_CFG_KSD_EN == DEF_ENABLED)
#include    <ksd.h>
#endif

#if (PROBE_COM_CFG_RS232_EN == DEF_ENABLED)
#include    <probe_rs232.h>
#endif

#if (APP_CFG_PROBE_OS_PLUGIN_EN == DEF_ENABLED)
#include    <os_probe.h>
#endif
#endif



#endif
