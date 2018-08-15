/**
  ******************************************************************************
  * @file    Project/ucosii stm32f4 mdk475/peripherial_init.h 
  * @author  Kuangjing
  * @version ucosii
  * @date    20180619
  * @brief   初始化各外设
  ******************************************************************************
  ******************************************************************************
  */




#ifndef		__PERIPHERIAL_INIT_H
#define		__PERIPHERIAL_INIT_H

#include	"stm32f4xx.h"




void	NVIC_Config(void);
void	GPIO_Config(void);
void	USART2_DMA_Config(void);
void	USART3_DMA_Config(void);
void	SPI1_DMA_Config(void);
void	Timer1_Config(void);
/*上下桥臂开关*/
void	TIM1_CH1_OFF_CH1N_ON(void);
void	TIM1_CH1_OFF_CH1N_OFF(void);
void	TIM1_CH1_ON_CH1N_OFF(void);
void	TIM1_CH1_OFF_CH1N_PWM(void);
void	TIM1_CH1_PWM_CH1N_OFF(void);

void	TIM1_CH2_OFF_CH2N_ON(void);
void	TIM1_CH2_OFF_CH2N_OFF(void);
void	TIM1_CH2_ON_CH2N_OFF(void);
void	TIM1_CH2_OFF_CH2N_PWM(void);
void	TIM1_CH2_PWM_CH2N_OFF(void);

void	TIM1_CH3_OFF_CH3N_ON(void);
void	TIM1_CH3_OFF_CH3N_OFF(void);
void	TIM1_CH3_ON_CH3N_OFF(void);
void	TIM1_CH3_OFF_CH3N_PWM(void);
void	TIM1_CH3_PWM_CH3N_OFF(void);

void	Timer2_Config(void);
void	Timer3_Config(void);
void	Timer4_Config(void);
void	ADC_DMA_Config(void);
void	CAN_Config(void);
void	BrakeControl(uint8_t	mode);
void	LED_ON(void);
void	LED_OFF(void);


#endif

/**********************end of file************************/


