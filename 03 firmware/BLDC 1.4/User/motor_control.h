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
#include	"global_parameters.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void		Hall_Start_Convert(void);

void		Hall_Runtime_Convert(void);

void		PWM_TIM_Start(void);

void		PWM_TIM_Halt(void);

void		CurrentLoopRefresh_TIM_Start(void);

void		CurrentLoopRefresh_TIM_Halt(void);

void		SpeedLoopRefresh_TIM_Start(void);

void		SpeedLoopRefresh_TIM_Halt(void);

void		PositionLoopRefresh_TIM_Start(void);

void		PositionLoopRefresh_TIM_Halt(void);

void		IR2130S_force_reset(void);

void		Current_PID_Cal(volatile PID_struct * pid);

void		Speed_PID_Cal(volatile PID_struct * pid);

void		Position_PID_Cal(volatile PID_struct * pid);

void		Read_Current_Bias(void);

void		CAN_SEND_CVP(void);



#endif







