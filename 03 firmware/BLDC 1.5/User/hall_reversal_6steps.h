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

void	TIM1_CH1_OFF_CH1N_ON(void);
void	TIM1_CH1_OFF_CH1N_OFF(void);
void	TIM1_CH1_ON_CH1N_OFF(void);
void	TIM1_CH1_OFF_CH1N_PWM(void);
void	TIM1_CH1_PWM_CH1N_OFF(void);
void	TIM1_CH1_PWM_CH1N_PWM(void);

void	TIM1_CH2_OFF_CH2N_ON(void);
void	TIM1_CH2_OFF_CH2N_OFF(void);
void	TIM1_CH2_ON_CH2N_OFF(void);
void	TIM1_CH2_OFF_CH2N_PWM(void);
void	TIM1_CH2_PWM_CH2N_OFF(void);
void	TIM1_CH2_PWM_CH2N_PWM(void);
	
void	TIM1_CH3_OFF_CH3N_ON(void);
void	TIM1_CH3_OFF_CH3N_OFF(void);
void	TIM1_CH3_ON_CH3N_OFF(void);
void	TIM1_CH3_OFF_CH3N_PWM(void);
void	TIM1_CH3_PWM_CH3N_OFF(void);
void	TIM1_CH3_PWM_CH3N_PWM(void);




extern	const		void	(*runtime_6steps_switch_table[2][8])();
extern	const		void	(*startup_6steps_switch_table[2][8])();

uint16_t	Hall_State_Read(void);


#endif



