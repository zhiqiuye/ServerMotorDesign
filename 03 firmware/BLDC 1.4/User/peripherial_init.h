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
#include	"stm32f4xx_can.h"

/* Private typedef -----------------------------------------------------------*/


void	NVIC_Config(void);
void	GPIO_Config(void);
void	USART2_DMA_Config(void);
void	USART3_DMA_Config(void);
void	SPI1_DMA_Config(void);
void	SPI_DMA_ReadData(uint16_t length);
void 	SPI_DMA_WriteData(uint16_t length);
void	Timer1_Config(void);
void	Timer2_Config(void);
void	Timer3_Config(void);
void	Timer4_Config(void);
void	Timer5_Config(void);
void	Timer8_Master_Config(void);
void	ADC1_DMA_Config(void);
void	CAN_Config(void);
uint8_t CAN1_TX_Data(volatile CanTxMsg * msg,uint32_t stdID,uint8_t * data,uint8_t dataLen);
void	BrakeControl(uint8_t	mode);
void	LED_GREEN_ON(void);
void	LED_GREEN_OFF(void);
void	LED_RED_ON(void);
void	LED_RED_OFF(void);

void	DAC_Config(void);


#endif

/**********************end of file************************/


