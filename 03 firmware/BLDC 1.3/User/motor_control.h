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
void		Hall_Convert(void);

//void		NULL_Switch(void);
//void		H1_L2(void);
//void		H1_L3(void);
//void		H2_L1(void);
//void		H2_L3(void);
//void		H3_L2(void);
//void		H3_L1(void);

void		PWM_TIM_Start(void);

void		PWM_TIM_Halt(void);

void		CurrentLoopRefresh_TIM_Start(void);

void		CurrentLoopRefresh_TIM_Halt(void);

void		SpeedPosLoopRefresh_TIM_Start(void);

void		SpeedPosLoopRefresh_TIM_Halt(void);

void		IR2130S_force_reset(void);

void		Curr_PID_Cal(volatile PID_Struct * pid);

void		Speed_PID_Cal(volatile PID_Struct * pid);

void		Read_Current_Bias(void);



#endif







