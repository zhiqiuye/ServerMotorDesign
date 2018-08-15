/**
  ******************************************************************************
  * @file    Project/user/motor_control.h 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170728
  * @brief   
  ******************************************************************************
  ******************************************************************************
  */


#ifndef		__MOTOR_CONTROL_
#define		__MOTOR_CONTROL_

#include	"stm32f4xx.h"



/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint16_t	Hall_State_Read(void);
void		Hall_Convert(void);
void		PWM_TIM_Start(void);
void		PWM_TIM_Halt(void);
void		CurrentLoopRefresh_TIM_Start(void);
void		CurrentLoopRefresh_TIM_Halt(void);
void		IR2130S_force_reset(void);





#endif







