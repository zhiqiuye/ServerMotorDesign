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
#include	"stm32f4xx_it.h"
#include	"stm32f4xx_dac.h"
#include	"stm32f4xx_spi.h"
#include	"delay.h"
#include	"peripherial_init.h"
#include	"motor_control.h"
#include	"global_parameters.h"
#include	"foc_reversal_svpwm.h"

/*Private Function---------------------------------------------------------------*/

uint8_t		rcv_data;


/*Public Function----------------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	函数名称			：StateMachine_Task(void * parg)
	参数含义			：null
	函数功能			：状态机管理
	----------------------------------------------------------------------------*/
void	StateMachine_Task(void * parg)
{
	(void) parg;

	while(1)
	{
		if(m_motor_ctrl.m_sys_state.u8_cur_state == Idle_state)
		{
			
		}
		else if(m_motor_ctrl.m_sys_state.u8_cur_state == Run_state)
		{
			
		}
		else if(m_motor_ctrl.m_sys_state.u8_cur_state == Prepare_state)
		{
			OSTimeDlyHMSM(0,0,0,100);
			Read_Current_Bias();															//电流采集电路电压偏置计算
			
			m_motor_ctrl.m_sys_state.u8_abs_encoder_used	=	1;							//启用绝对值编码器

			OSTimeDlyHMSM(0,0,0,100);
			
			BrakeControl(1);																//开启抱闸
			
			Hall_Start_Convert();															//开始初次相位识别					
			
//			PositionLoopRefresh_TIM_Start();
//			
//			SpeedLoopRefresh_TIM_Start();													//开启速度位置环更新定时器

			CurrentLoopRefresh_TIM_Start();													//开启电流环更新

			m_motor_ctrl.m_sys_state.u8_cur_state 	=	Run_state;
			m_motor_ctrl.m_sys_state.u8_pre_state	=	Prepare_state;
		}
		else if(m_motor_ctrl.m_sys_state.u8_cur_state == Halt_state)
		{
		
		}
		OSTimeDlyHMSM(0,0,0,20);
	}
}



	/*---------------------------------------------------------------------------
	函数名称			：LED_Task(void * parg)
	参数含义			：null
	函数功能			：闪烁显示
	----------------------------------------------------------------------------*/
void	LED_Task(void * parg)
{
	uint8_t		data[2]={0x55,0xAA};
	
	
	(void)	parg;
	while(1)
	{
		LED_GREEN_OFF();
		OSTimeDlyHMSM(0,0,0,800);
		LED_GREEN_ON();
		OSTimeDlyHMSM(0,0,0,200);

		rcv_data = data[0] - data[1];
//CAN发送报文测试
//		while(!CAN1_TX_Data((CanTxMsg*)&(m_can.TxMessage),
//							0x0000,
//							(uint8_t*)&data,
//							2));
	}
}



	/*---------------------------------------------------------------------------
	函数名称			：485_Task(void * parg)
	参数含义			：null
	函数功能			：闪烁显示
	----------------------------------------------------------------------------*/
void	RS485_Task(void * parg)
{
	uint16_t	tamp_cnt=0;
	uint16_t	dac_value=0;	
	(void)	parg;

	while(1)
	{
		tamp_cnt = OSCPUUsage;

/*		dac_value +=10;
		if(dac_value >=4096)
			dac_value	=	0;
		
		DAC_SetChannel1Data(DAC_Align_12b_R,dac_value);*/
		
		OSTimeDlyHMSM(0,0,1,0);
	}
}


	/*---------------------------------------------------------------------------
	函数名称			：_Task(void * parg)
	参数含义			：null
	函数功能			：闪烁显示
	----------------------------------------------------------------------------*/




/****************************end of file************************************/
