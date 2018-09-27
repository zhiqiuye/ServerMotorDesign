/**
  ******************************************************************************
  * @file    Project/user/delay.c 
  * @author  Kuangjing
  * @version ucosii 2.92
  * @date    20170724
  * @brief   使用systick进行延时
  ******************************************************************************
  ******************************************************************************
  */


#ifndef	__DELAY_H_
#define	__DELAY_H_

#include	"stm32f4xx.h"



/*sysTick 定义的寄存器*/
#define  OS_CPU_CM4_NVIC_ST_CTRL    (*((volatile uint32_t *)0xE000E010uL)) /* SysTick Ctrl & Status Reg. */
#define  OS_CPU_CM4_NVIC_ST_RELOAD  (*((volatile uint32_t *)0xE000E014uL)) /* SysTick Reload  Value Reg. */
#define  OS_CPU_CM4_NVIC_ST_CURRENT (*((volatile uint32_t *)0xE000E018uL)) /* SysTick Current Value Reg. */
#define  OS_CPU_CM4_NVIC_ST_CAL     (*((volatile uint32_t *)0xE000E01CuL)) /* SysTick Cal     Value Reg. */
#define  OS_CPU_CM4_NVIC_SHPRI1     (*((volatile uint32_t *)0xE000ED18uL)) /* System Handlers  4 to  7 Prio.       */
#define  OS_CPU_CM4_NVIC_SHPRI2     (*((volatile uint32_t *)0xE000ED1CuL)) /* System Handlers  8 to 11 Prio.       */
#define  OS_CPU_CM4_NVIC_SHPRI3     (*((volatile uint32_t *)0xE000ED20uL)) /* System Handlers 12 to 15 Prio.       */





void	delay_us(uint32_t	dlyTime);


#endif






















