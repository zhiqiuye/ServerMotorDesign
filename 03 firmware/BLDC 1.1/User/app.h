/**
  ******************************************************************************
  * @file    Project/user/app.h 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170728
  * @brief   ����������
  ******************************************************************************
  ******************************************************************************
  */



#ifndef		__APP_H
#define		__APP_H
/* Includes ------------------------------------------------------------------*/
#include	"peripherial_init.h"
#include	"stm32f4xx.h"
#include	"stm32f4xx_tim.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void	LED_Task(void * parg);
void	RS485_Task(void * parg);


#endif














