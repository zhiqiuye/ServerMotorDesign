/**
  ******************************************************************************
  * @file    Project/user/stm32f4xx_it.c 
  * @author  Kuangjing
  * @version V1.7.1
  * @date    20180801
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include	"stm32f4xx_it.h"
#include	"stm32f4xx_tim.h"
#include	"stm32f4xx_gpio.h"
#include	"stm32f4xx_usart.h"
#include	"stm32f4xx_dma.h"
#include	"global_parameters.h"
#include	"peripherial_init.h"
#include	"motor_control.h"
#include	"ucos_ii.h"
#include	"includes.h"
#include	"current_filter.h"
#include	"math.h"										//�������ʹ��ͷ�ļ�
#include	"arm_math.h"
#include	"jingle_math.h"
#include	"spd_pos_filter.h"
#include	"hall_reversal_6steps.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define			USE_CURRENT_TRACE							//ʹ�õ���ģ�����
#define			USE_SPEED_TRACE								//ʹ���ٶȻ�ģ�����
//#define			USE_POSITION_TRACE							//ʹ��λ�û�ģ�����

/* Private macro -------------------------------------------------------------*/
#define			RADIANS				1.047197533333f			//����1����
#define			_2PI				6.283185307179586476925286766559f
#define			_PI					3.1415926535897932384626433832795f
#define			_0_5PI				1.5707963267948966192313216916398f

/* Private variables ---------------------------------------------------------*/
#ifdef	USE_CURRENT_TRACE
uint32_t		step_cnt			=	0;					//������������
uint32_t		sim_trace_current	=	0;					//�����õ�Ŀ�����ֵ
float32_t		f_sin				=	0.0f;				//����
#define			CURRENT_AMP			0.5f
#define			CURRENT_FQC			4.0f
#define			CURRENT_CYC			(20000.0f/CURRENT_FQC)
#endif

#ifdef	USE_SPEED_TRACE
uint32_t		step_cnt			=	0;					//������������
uint32_t		sim_trace_current	=	0;					//�����õ�Ŀ�����ֵ
float32_t		f_sin				=	0.0f;				//����
#define			SPEED_AMP			1.5f
#define			SPEED_FQC			1.0f
#define			SPEED_CYC			(1000.0f/SPEED_FQC)
#endif

#ifdef	USE_POSITION_TRACE
uint32_t		step_cnt			=	0;					//������������
uint32_t		sim_trace_current	=	0;					//�����õ�Ŀ�����ֵ
float32_t		f_sin				=	0.0f;				//����
#define			POS_AMP				10000.0f
#define			POS_FQC				0.5f
#define			POS_CYC				(1000.0f/POS_FQC)
#endif

float		ft;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
	/*---------------------------------------------------------------------------
	��������			��TIM1_UP_TIM10_IRQHandler(void)
	��������			��null
	��������			��	PWM����������жϣ�����Ƶ����40KHz����PWMƵ�ʵ�2��
							��PWM��������������ж��д�ADC������������ʱ7us;
							
							���Ҹ��µ�����:
							20Khz��50us���ڣ������У�����ʱ��1.5us							
	----------------------------------------------------------------------------*/
void	TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET)
	{
		/*����������������ж�ʱ����ADC�ɼ�*/
		if(TIM1->CR1 & 0x0010)
		{
//			GPIOC->ODR	|=	0x0001;															//�ø�C1
			/*����������������£������������������£������������ʱ��1.3us*/
			if(m_motor_ctrl.u8_is_currloop_used ==	1)
			{
#ifdef	USE_CURRENT_TRACE	/*����Ŀ�����ҵ����źţ�����ʱ��4us*/
				f_sin	=	1.0f + CURRENT_AMP * arm_sin_f32( (float32_t)step_cnt * _2PI / CURRENT_CYC );
				step_cnt++;
				if(step_cnt >= (uint32_t)CURRENT_CYC)
					step_cnt = 0;
				m_motor_ctrl.f_set_current						=	0.6;//f_sin;
				m_motor_ctrl.u8_current_set_data_refreshed		=	1;
				if(step_cnt == 0)
					GPIOC->ODR	|=	0x0001;
				if(step_cnt == (uint32_t)(CURRENT_CYC/2.0f))
					GPIOC->ODR	&=	0xFFFE;
#endif
				/*����ֵ���º󣬽����������DMA�ж�*/
				if(m_motor_ctrl.u8_current_read_data_refreshed	==	1)
				{
					Current_PID_Cal(&(m_current_pid.curr_pid));											//��ˢ�·�������ֵ�����������³��򲿷�
					m_motor_ctrl.u8_current_read_data_refreshed	=	0;									
				}
			}
//			GPIOC->ODR	&=	0xFFFE;		
		}
		else
		{
			/*����ADC DMA*/
			ADC_SoftwareStartConv(ADC1);																//����ADC����		
			/*�ٶȻ����ݸ��º󣬸���Ŀ�����*/
			if(m_motor_ctrl.u8_current_set_data_refreshed	== 1)
			{
				m_current_pid.curr_pid.Ref_In	=	(float)fabs((double)m_motor_ctrl.f_set_current);	//����ֵȡ����ֵ
				m_motor_ctrl.u8_current_set_data_refreshed	=	0;
			}
		}
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);														//���TIM1�жϱ�־
	}
}


	/*---------------------------------------------------------------------------
	��������			��	TIM2_IRQHandler(void)
	��������			��	null
	��������			��	��ʱ�����ٶ�λ�û���1Khz��1ms���ڣ������У�
							
							�ٶȻ����º�ʱ	3.5us
	----------------------------------------------------------------------------*/
void	TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
	{
//		GPIOC->ODR	|=	0x0001;			
		Read_IncEncoder();																//��ȡ���������ݣ���ʱ1us
//		GPIOC->ODR	&=	0xFFFE;	
		
		/*����λ�û�*/
		if(m_motor_ctrl.u8_is_posloop_used	==	1)
		{
			
#ifdef	USE_POSITION_TRACE	/*����Ŀ������λ���źţ�*/
			f_sin	=	POS_AMP * arm_sin_f32((float32_t)step_cnt * _2PI / POS_CYC );
			step_cnt++;
			if(step_cnt >= (uint32_t)POS_CYC)
				step_cnt = 0;
			m_motor_ctrl.f_set_position						=	f_sin;
			m_motor_ctrl.u8_position_set_data_refreshed		=	1;
			if(step_cnt == 0)
				GPIOC->ODR	|=	0x0001;
			if(step_cnt == (uint32_t)(POS_CYC/2.0f))
				GPIOC->ODR	&=	0xFFFE;
#endif
			
			if(m_motor_ctrl.u8_position_set_data_refreshed	==	1)
			{
				m_position_pid.pos_pid.Ref_In				=	m_motor_ctrl.f_set_position;
				
				Position_PID_Cal(&(m_position_pid.pos_pid));
				m_motor_ctrl.u8_position_set_data_refreshed	=	0;
			}
		}

		
		/*�����ٶȻ�*/
		if(m_motor_ctrl.u8_is_speedloop_used	==	1)
		{
			if(m_motor_ctrl.u8_speed_read_data_refreshed	==	1)						//�����ٶȻ�pid����
			{
				
#ifdef	USE_SPEED_TRACE	/*����Ŀ�������ٶ��źţ�����ʱ��4us*/
				f_sin	=	SPEED_AMP * arm_sin_f32( (float32_t)step_cnt * _2PI / SPEED_CYC );
				step_cnt++;
				if(step_cnt >= (uint32_t)SPEED_CYC)
					step_cnt = 0;
				m_motor_ctrl.f_set_speed					=	f_sin;
				m_motor_ctrl.u8_speed_set_data_refreshed	=	1;
				if(step_cnt == 0)
					GPIOC->ODR	|=	0x0001;
				if(step_cnt == (uint32_t)(SPEED_CYC/2.0f))
					GPIOC->ODR	&=	0xFFFE;
#endif
				Speed_PID_Cal(&(m_speed_pid.spd_pid));
				m_motor_ctrl.u8_speed_read_data_refreshed	=	0;						//ʹ���������ٶ�ֵ�󣬽���־λ�õ�
			}				
			if(m_motor_ctrl.u8_speed_set_data_refreshed		==	1)						//���²ο��ٶ�ֵ
			{
				m_speed_pid.spd_pid.Ref_In					=	m_motor_ctrl.f_set_speed;	//�������ٶ�ֵֵд��PID����
				m_motor_ctrl.u8_speed_set_data_refreshed	=	0;
				m_motor_ctrl.u8_current_set_data_refreshed	=	1;						//�ٶȻ����±�־λ
			}
		}
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);										//����жϱ�־λ
	}
}



	/*---------------------------------------------------------------------------
	��������			��	TIM3_IRQHandler(void)
	��������			��	null
	��������			��	TIM3������ģʽ����жϣ��Լ�I�����벶���ж�							
	----------------------------------------------------------------------------*/
void	TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET)										//AB���������ж�
	{
	}
	
	if(TIM_GetITStatus(TIM3,TIM_IT_CC3) != RESET)											//I�����벶���ж�
	{
		
	}
	
	TIM_ClearITPendingBit(TIM3,TIM_IT_CC3|TIM_IT_Update);									//����ж�
}




	/*---------------------------------------------------------------------------
	��������			��TIM4_IRQHandler(void)
	��������			��null
	��������			������Hall�������źŽ��л���
	----------------------------------------------------------------------------*/
void	TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_CC1)!=RESET)
	{
		Hall_Start_Convert();																//��������
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC1);
	}
}



	/*---------------------------------------------------------------------------
	��������			��TIM5_IRQHandler(void)
	��������			��null
	��������			��
	----------------------------------------------------------------------------*/
void	TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_CC1)!=RESET)
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_CC1);
	}
}






	/*---------------------------------------------------------------------------
	��������			��DMA2_Stream0_IRQHandler(void)
	��������			��null
	��������			��	ADC-DMA��������жϣ���PWM�ߵ͵�ƽ�м�λ�ÿ�ʼ����������6us������
							4��ֵ�˲�����ʱ��Ϊ1.25us��

							����BLDC�ĵ�����Ӳ����·����ÿ�����ű�һ���������裬
							�����ű۸ı�ͨ·ʱ����Ӧ�����ű�������������Ӧ���ű۵ĵ�������ͻ�䣬
							ʹ��ǰ���ű���Ȼ�е�����
							����ʵ��bldc�ĵ���ֵΪ��·��������ֵ�ĺͳ���2

	----------------------------------------------------------------------------*/
void	DMA2_Stream0_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0))
	{
//		GPIOC->ODR	|=	0x0001;	
		Current_Average_X8_Filter(&m_motor_rt_para);												//��ԭʼ�������ݽ��л��������˲����Լ�ͻ����޳�
		
		m_motor_rt_para.f_adc_UVW_I					=	(float)(m_motor_rt_para.u16_uvw_current)*0.0100708f;
		
//		GPIOC->ODR	&=	0xFFFE;
		DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
	}
}




/******************************************************************************
*			����˵����	CAN1_RX0_IRQHandler(void)
*			����˵����	��
*			ʹ�÷�Χ��	CAN1  FIFO0 �Ľ����ж�
*******************************************************************************/
void	CAN1_RX0_IRQHandler(void)
{
	OSIntEnter();
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0) == SET)								//FIFO 0 message pending Interrupt,FIFO0�����ж�
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
		/*���н��մ���*/
		
		CAN_Receive(CAN1,CAN_FIFO0,(CanRxMsg *)&(m_can.RxMessage));			//���պ���
		/*����HeartBeat���ģ���DriverCAN������λ����*/
		if((m_can.RxMessage.StdId & 0xF80) == 0x580 )						//֡Ϊ���ݷ���֡
		{
			/*����Ƕ�ȡ�ٶ�ֵ�Ļظ�����	0x60FF*/
			if((m_can.RxMessage.Data[0] == 0x43) && (m_can.RxMessage.Data[1] == 0xFF) && (m_can.RxMessage.Data[2] == 0x60))
			{

			}
			/*����Ƕ�ȡλ��ֵ�Ļظ�����	0x2240*/
			else if((m_can.RxMessage.Data[0] == 0x43) && (m_can.RxMessage.Data[1] == 0x40) && (m_can.RxMessage.Data[2] == 0x22))
			{

			}
			else
				m_can.CAN_msg_recv_flag		=	1;
		}
		else																	//���յ�heartbeat�󣬶�heartbeat_refreshed_flag��λ
		{
			/*ͨ��ID�����������ĵ�����*/
			if(m_can.RxMessage.StdId == (0x700 ))			//������1��heartbeat
			{

			}
			else if(m_can.RxMessage.StdId == (0x700 ))		//������2��heartbeat
			{

			}
			else if(m_can.RxMessage.StdId == (0x700 ))		//������3��heartbeat
			{

			}
			else if(m_can.RxMessage.StdId == (0x700 ))		//������4��heartbeat
			{

			}
			/*ͨ��ID�Լ����ֽ����ݣ�ȷ���ڵ�״̬*/
			/*0x04  ֹͣ״̬��0x05  ����״̬��0x7F  Ԥ����״̬*/
			if(((m_can.RxMessage.StdId & 0xF00) == 0x700) && (m_can.RxMessage.Data[0] == 0x7F))
			{
				
			}
		}
	}
	if(CAN_GetITStatus(CAN1,CAN_IT_FF0) == SET)									//FIFO 0 full Interrupt,FIFO0��������
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_FF0);	
		/*����FIFO����*/
		//
		//
	}
	OSIntExit();
}




/******************************************************************************
*			����˵����	CAN1_TX_IRQHandler(void)
*			����˵����	��
*			ʹ�÷�Χ��	CAN1  FIFO0 �Ľ����ж�
*******************************************************************************/
void	CAN1_TX_IRQHandler(void)
{
	OSIntEnter();
	if(m_can.CAN_msg_num[0])
	{
		if(CAN1->TSR & CAN_TSR_RQCP0)									//��������0��������ж�, RQCP0
		{
			CAN1->TSR 					=	CAN_TSR_RQCP0;				//д��1����
			CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);
			m_can.CAN_msg_num[0]		=	0;			
		}
	}
	else if(m_can.CAN_msg_num[1])
	{
		if(CAN1->TSR & CAN_TSR_RQCP1)									//��������1��������ж�, RQCP1
		{
			CAN1->TSR 					=	CAN_TSR_RQCP1;				//д��1����
			CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);
			m_can.CAN_msg_num[1]		=	0;
		}
	}
	else if(m_can.CAN_msg_num[2])
	{
		if(CAN1->TSR & CAN_TSR_RQCP2)									//��������2��������ж�, RQCP2
		{
			CAN1->TSR 					=	CAN_TSR_RQCP2;				//д��1����
			CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);
			m_can.CAN_msg_num[2]		=	0;
		}
	}
	OSIntExit();
}


//-----------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------
//NOT USED
//
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
