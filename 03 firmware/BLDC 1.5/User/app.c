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
#include	"hall_reversal_svpwm.h"

/*Private Function---------------------------------------------------------------*/

uint8_t		rcv_data;


/*Public Function----------------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	��������			��StateMachine_Task(void * parg)
	��������			��null
	��������			��״̬������
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
			Read_Current_Bias();															//�����ɼ���·��ѹƫ�ü���
			
			m_motor_ctrl.m_sys_state.u8_abs_encoder_used	=	1;							//���þ���ֵ������

			OSTimeDlyHMSM(0,0,0,100);
			
			BrakeControl(1);																//������բ
			
			Hall_Start_Convert();															//��ʼ������λʶ��					
			
//			PositionLoopRefresh_TIM_Start();
//			
//			SpeedLoopRefresh_TIM_Start();													//�����ٶ�λ�û����¶�ʱ��

			CurrentLoopRefresh_TIM_Start();													//��������������

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
	��������			��LED_Task(void * parg)
	��������			��null
	��������			����˸��ʾ
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
//CAN���ͱ��Ĳ���
//		while(!CAN1_TX_Data((CanTxMsg*)&(m_can.TxMessage),
//							0x0000,
//							(uint8_t*)&data,
//							2));
	}
}



	/*---------------------------------------------------------------------------
	��������			��485_Task(void * parg)
	��������			��null
	��������			����˸��ʾ
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
	��������			��_Task(void * parg)
	��������			��null
	��������			����˸��ʾ
	----------------------------------------------------------------------------*/




/****************************end of file************************************/
