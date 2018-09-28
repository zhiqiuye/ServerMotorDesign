/**
  ******************************************************************************
  * @file    Project/user/hallreversal.h
  * @author  Kuangjing
  * @version ucosii
  * @date    20170928
  * @brief   霍尔换向以及SVPWM操作
  ******************************************************************************
  ******************************************************************************
  */
  
  
#ifndef		__HALL_REVERSAL_SVPWM_H
#define		__HALL_REVERSAL_SVPWM_H


#include	"stm32f4xx.h"
#include	"stm32f4xx_gpio.h"
#include	"stm32f4xx_tim.h"
#include	"global_parameters.h"



extern	void	(*runtime_svpwm_switch_table[2][8])();


void	U_OFF_V_OFF_W_OFF(void);
void	U_ON_V_OFF_W_OFF(void);
void	U_OFF_V_ON_W_OFF(void);
void	U_ON_V_ON_W_OFF(void);
void	U_OFF_V_OFF_W_ON(void);
void	U_ON_V_OFF_W_ON(void);
void	U_OFF_V_ON_W_ON(void);
void	U_ON_V_ON_W_ON(void);






#endif

/**************************************END OF FILE*****************************************/

