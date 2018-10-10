/**
  ******************************************************************************
  * @file    Project/user/foc_reversal_svpwm.h
  * @author  Kuangjing
  * @version ucosii
  * @date    20170928
  * @brief   ���������Լ�SVPWM����
  ******************************************************************************
  ******************************************************************************
  */
  
  
#ifndef		__FOC_REVERSAL_SVPWM_H
#define		__FOC_REVERSAL_SVPWM_H


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

void	RotorCorrection(void);
void	RotorRecognition(uint8_t hall_state);
void	FOC_Cal(void);



#endif

/**************************************END OF FILE*****************************************/

