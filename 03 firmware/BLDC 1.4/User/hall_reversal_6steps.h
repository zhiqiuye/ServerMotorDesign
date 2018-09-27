/**
  ******************************************************************************
  * @file    Project/user/hallreversal.h 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170509
  * @brief   霍尔换向表以及对应换向寄存器操作
  ******************************************************************************
  ******************************************************************************
  */

#ifndef		__HALL_REVERSAL_6STEPS_
#define		__HALL_REVERSAL_6STEPS_

#include	"stm32f4xx.h"
#include	"global_parameters.h"

extern	const	void	(*runtime_switch_table[2][8])()	;
extern	const	void	(*startup_switch_table[2][8])();

uint16_t	Hall_State_Read(void);


#endif



