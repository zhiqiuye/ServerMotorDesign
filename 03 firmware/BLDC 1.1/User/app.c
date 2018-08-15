/**
  ******************************************************************************
  * @file    Project/ucosii stm32f4 mdk475/app.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170509
  * @brief   applications.
  ******************************************************************************
  ******************************************************************************
  */

#include	"app.h"
#include	"includes.h"
#include	"stm32f4xx_tim.h"
#include	"delay.h"
#include	"peripherial_init.h"
#include	"stm32f4xx_it.h"
#include	"motor_control.h"

	/*---------------------------------------------------------------------------
	函数名称			：LED_Task(void * parg)
	参数含义			：null
	函数功能			：闪烁显示
	----------------------------------------------------------------------------*/
void	LED_Task(void * parg)
{
	(void)	parg;
	while(1)
	{
		LED_OFF();
		OSTimeDlyHMSM(0,0,0,800);
		LED_ON();
		OSTimeDlyHMSM(0,0,0,50);
	}
}



	/*---------------------------------------------------------------------------
	函数名称			：485_Task(void * parg)
	参数含义			：null
	函数功能			：闪烁显示
	----------------------------------------------------------------------------*/
void	RS485_Task(void * parg)
{
	(void)	parg;
	uint16_t	tamp_cnt = 5000;
//	
//	TIM_Cmd(TIM1,ENABLE);
//	
//	TIM1_CH1_OFF_CH1N_OFF();
//	TIM1_CH2_OFF_CH2N_OFF();
//	TIM1_CH3_OFF_CH3N_OFF();
//	
	while(1)
	{
//		TIM_Cmd(TIM1,ENABLE);
//		TIM_CtrlPWMOutputs(TIM1,ENABLE);
//		//启动任务
//		if(tamp_cnt>=2000)
//		{		
//			tamp_cnt -= 10;
//			
//			//逐渐增加占空比20180727
//			TIM1->CCR1	+=	10;
//			TIM1->CCR2	+=	10;
//			TIM1->CCR3	+=	10;
//			TIM_Cmd(TIM1,ENABLE);
//			delay_us(tamp_cnt<<2);
//			//L1-pwm	H2-on
//			TIM1_CH1_OFF_CH1N_PWM();			//1on
//			TIM1_CH2_ON_CH2N_OFF();
//			TIM1_CH3_OFF_CH3N_OFF();
//		
//			delay_us(tamp_cnt<<2);
//			//L1-on		H3-pwm
//			TIM1_CH1_OFF_CH1N_ON();				//
//			TIM1_CH2_OFF_CH2N_OFF();
//			TIM1_CH3_PWM_CH3N_OFF();
//			
//			delay_us(tamp_cnt<<2);
//			//L2-pwm	H3-on
//			TIM1_CH1_OFF_CH1N_OFF();
//			TIM1_CH2_OFF_CH2N_PWM();			
//			TIM1_CH3_ON_CH3N_OFF();
//			
//			delay_us(tamp_cnt<<2);
//			//H1-pwm	L2-on
//			TIM1_CH1_PWM_CH1N_OFF();			
//			TIM1_CH2_OFF_CH2N_ON();
//			TIM1_CH3_OFF_CH3N_OFF();
//			
//			delay_us(tamp_cnt<<2);
//			//H1-on		L3-pwm
//			TIM1_CH1_ON_CH1N_OFF();
//			TIM1_CH2_OFF_CH2N_OFF();
//			TIM1_CH3_OFF_CH3N_PWM();
//			
//			delay_us(tamp_cnt<<2);
//			//H2-pwm	L3-on
//			TIM1_CH1_OFF_CH1N_OFF();
//			TIM1_CH2_PWM_CH2N_OFF();
//			TIM1_CH3_OFF_CH3N_ON();
//		}
//		else if(tamp_cnt>0 && tamp_cnt<2000)
//		{	
//			TIM1->CCR1	=	2000;
//			TIM1->CCR2	=	2000;
//			TIM1->CCR3	=	2000;
//			TIM_ClearFlag(TIM4,TIM_IT_CC1);
//			TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);	
//			TIM_Cmd(TIM4,ENABLE);
//			
//			Hall_State_Read();
//			Hall_Convert();			
//			
//			tamp_cnt=0;			
//		}
//		else if(tamp_cnt==0)
//			;

		OSTimeDlyHMSM(0,0,0,10);
	}
}








	/*---------------------------------------------------------------------------
	函数名称			：_Task(void * parg)
	参数含义			：null
	函数功能			：闪烁显示
	----------------------------------------------------------------------------*/


	/*---------------------------------------------------------------------------
	函数名称			：_Task(void * parg)
	参数含义			：null
	函数功能			：闪烁显示
	----------------------------------------------------------------------------*/


	/*---------------------------------------------------------------------------
	函数名称			：_Task(void * parg)
	参数含义			：null
	函数功能			：闪烁显示
	----------------------------------------------------------------------------*/



/****************************end of file************************************/
