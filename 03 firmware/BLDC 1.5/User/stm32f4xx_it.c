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
#include	"hall_reversal_svpwm.h"
#include	"peripherial_init.h"
#include	"node_can_config.h"




/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define			USE_CURRENT_TRACE							//ʹ�õ���ģ�����
//#define			USE_SPEED_TRACE								//ʹ���ٶȻ�ģ�����
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
#define			CURRENT_AMP			1.0f
#define			CURRENT_FQC			1.0f
#define			CURRENT_CYC			(20000.0f/CURRENT_FQC)
#endif

#ifdef	USE_SPEED_TRACE
uint32_t		step_cnt			=	0;					//������������
uint32_t		sim_trace_current	=	0;					//�����õ�Ŀ�����ֵ
float32_t		f_sin				=	0.0f;				//����
#define			SPEED_AMP			1.5f
#define			SPEED_FQC			25.0f
#define			SPEED_CYC			(1000.0f/SPEED_FQC)
#endif

#ifdef	USE_POSITION_TRACE
uint32_t		step_cnt			=	0;					//������������
uint32_t		sim_trace_current	=	0;					//�����õ�Ŀ�����ֵ
float32_t		f_sin				=	0.0f;				//����
#define			POS_AMP				2.0f
#define			POS_FQC				1.0f
#define			POS_CYC				(1000.0f/POS_FQC)
#endif

uint8_t			tim8_cnts = 0;


/* Private function prototypes -----------------------------------------------*/
//			GPIOC->ODR	|=	0x0001;				//�ø�C1
//			GPIOC->ODR	&=	0xFFFE;		
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
	/*������/�������------------------------------------------------------------
	��������			��TIM1_UP_TIM10_IRQHandler(void)
	��������			��null
	��������			��	PWM����������жϣ�����Ƶ����40KHz����PWMƵ�ʵ�2��
							��PWM��������������ж��д�ADC������������ʱ7us;
							
							���Ҹ��µ�����:
							20Khz��50us���ڣ������У�����ʱ��1.5us							
	----------------------------------------------------------------------------*/
void	TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM1->SR & TIM_IT_Update)																		//TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET)
	{
		/*����������������ж�ʱ����ADC�ɼ�*/
		if(TIM1->CR1 & 0x0010)
		{
			/*����������������£������������������£������������ʱ��1.3us*/
			if(m_motor_ctrl.m_motion_ctrl.u8_is_currloop_used ==	1)
			{
#ifdef	USE_CURRENT_TRACE	/*����Ŀ�����ҵ����źţ�����ʱ��4us*/
				f_sin	=	1.0f + CURRENT_AMP * arm_sin_f32( (float32_t)step_cnt * _2PI / CURRENT_CYC );
				step_cnt++;
				if(step_cnt >= (uint32_t)CURRENT_CYC)
					step_cnt = 0;
//				m_motor_ctrl.m_motion_ctrl.f_set_current						=	1.2f;//f_sin;
				m_motor_ctrl.m_motion_ctrl.u8_current_set_data_refreshed		=	1;
				if(step_cnt == 0)
				{
					GPIOC->ODR	|=	0x0001;
					m_motor_ctrl.m_motion_ctrl.f_set_current					=	0.0f;
				}
				if(step_cnt == (uint32_t)(CURRENT_CYC/2.0f))
				{
					GPIOC->ODR	&=	0xFFFE;
					m_motor_ctrl.m_motion_ctrl.f_set_current					=	0.6f;
				}
#endif
				/*����ֵ���º󣬽����������DMA�ж�*/
				if(m_motor_ctrl.m_motion_ctrl.u8_current_read_data_refreshed	==	1)
				{
					if(m_motor_ctrl.m_sys_state.u8_use_svpwm	==	NOT_USE_FOC)						//ʹ���������λ���
						Current_PID_Cal(&(m_pid.curr));													//��ˢ�·�������ֵ�����������³��򲿷�
					else																				//ʹ��FOC
						FOC_Cal();
					m_motor_ctrl.m_motion_ctrl.u8_current_read_data_refreshed	=	0;									
				}
			}
		}
		else
		{
			/*����ADC DMA*/
			ADC_SoftwareStartConv(ADC1);																//����ADC������������ɼ�
			ADC_SoftwareStartConv(ADC2);																//����ADC���������ش�����
			
			/*�ٶȻ����ݸ��º󣬸���Ŀ�����*/
			if(m_motor_ctrl.m_motion_ctrl.u8_current_set_data_refreshed			== 1)
			{
				if(m_motor_ctrl.m_sys_state.u8_use_svpwm	==	NOT_USE_FOC)							//ʹ���������λ���
					m_pid.curr.Ref_In											=	(float)fabs((double)m_motor_ctrl.m_motion_ctrl.f_set_current);	//����ֵȡ����ֵ
				else																					//ʹ��FOC
				{
					m_pid.iq.Ref_In												=	0.6f;
					m_pid.id.Ref_In												=	0.0f;
				}
					
				m_motor_ctrl.m_motion_ctrl.u8_current_set_data_refreshed		=	0;
			}
		}
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);														//���TIM1�жϱ�־
	}
}


	/*�ٶȻ�/λ�û�����----------------------------------------------------------
	��������			��	TIM2_IRQHandler(void)
	��������			��	null
	��������			��	��ʱ�����ٶ�λ�û���1Khz��1ms���ڣ������У�
							TIM2��ʱ����TIM8����
							�ٶȻ����º�ʱ	3.5us
	----------------------------------------------------------------------------*/
void	TIM2_IRQHandler(void)
{
	if(TIM2->SR & TIM_IT_Update)														//TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
	{
		tim8_cnts	=	0;
		Read_IncEncoder();																//��ȡ���������ݣ���ʱ1us
		if(m_motor_ctrl.m_sys_state.u8_abs_encoder_used)
			Read_AbsEncoder();															//����SSI��ȡ����
		
		/*����λ�û�*/
		if(m_motor_ctrl.m_motion_ctrl.u8_is_posloop_used	==	1)
		{
#ifdef	USE_POSITION_TRACE	/*����Ŀ������λ���źţ�*/
			f_sin	=	POS_AMP * arm_sin_f32((float32_t)step_cnt * _2PI / POS_CYC );
			step_cnt++;
			if(step_cnt >= (uint32_t)POS_CYC)
				step_cnt = 0;
			m_motor_ctrl.m_motion_ctrl.f_set_position						=	f_sin;//POS_AMP + f_sin;
			m_motor_ctrl.m_motion_ctrl.u8_position_set_data_refreshed		=	1;
			if(step_cnt == 0)
				GPIOC->ODR	|=	0x0001;
			if(step_cnt == (uint32_t)(POS_CYC/2.0f))
				GPIOC->ODR	&=	0xFFFE;
#endif
			
			if(m_motor_ctrl.m_motion_ctrl.u8_position_set_data_refreshed	==	1)
			{
				m_pid.pos.Ref_In				=	m_motor_ctrl.m_motion_ctrl.f_set_position + m_motor_rt_para.m_abs_encoder.f_abs_pos_init;
				Position_PID_Cal(&(m_pid.pos));
				m_motor_ctrl.m_motion_ctrl.u8_position_set_data_refreshed	=	0;
			}
		}
		
		/*�����ٶȻ�*/
		if(m_motor_ctrl.m_motion_ctrl.u8_is_speedloop_used	==	1)
		{
			if(m_motor_ctrl.m_motion_ctrl.u8_speed_read_data_refreshed	==	1)						//�����ٶȻ�pid����
			{	
#ifdef	USE_SPEED_TRACE	/*����Ŀ�������ٶ��źţ�����ʱ��4us*/
				f_sin	=	SPEED_AMP * arm_sin_f32( (float32_t)step_cnt * _2PI / SPEED_CYC );
				step_cnt++;
				if(step_cnt >= (uint32_t)SPEED_CYC)
					step_cnt = 0;
				m_motor_ctrl.m_motion_ctrl.f_set_speed					=	f_sin;
				m_motor_ctrl.m_motion_ctrl.u8_speed_set_data_refreshed	=	1;
				if(step_cnt == 0)
					GPIOC->ODR	|=	0x0001;
				if(step_cnt == (uint32_t)(SPEED_CYC/2.0f))
					GPIOC->ODR	&=	0xFFFE;
#endif
				Speed_PID_Cal(&(m_pid.spd));
				m_motor_ctrl.m_motion_ctrl.u8_speed_read_data_refreshed		=	0;						//ʹ���������ٶ�ֵ�󣬽���־λ�õ�
			}				
			if(m_motor_ctrl.m_motion_ctrl.u8_speed_set_data_refreshed		==	1)						//���²ο��ٶ�ֵ
			{
				m_pid.spd.Ref_In											=	m_motor_ctrl.m_motion_ctrl.f_set_speed;	//�������ٶ�ֵֵд��PID����
				m_motor_ctrl.m_motion_ctrl.u8_speed_set_data_refreshed		=	0;
				m_motor_ctrl.m_motion_ctrl.u8_current_set_data_refreshed	=	1;						//�ٶȻ����±�־λ
			}
		}
		
		CAN_SEND_CVP();																					//can���͵����ٶ�λ��ֵ
		
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);														//����жϱ�־λ
	}
}



	/*�������������-------------------------------------------------------------
	��������			��	TIM3_IRQHandler(void)
	��������			��	null
	��������			��	TIM3������ģʽ����жϣ��Լ�I�����벶���ж�							
	----------------------------------------------------------------------------*/
void	TIM3_IRQHandler(void)
{
	if(TIM3->SR & TIM_IT_Update)															//TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET)		//AB���������ж�
	{
	}
	
	if(TIM3->SR & TIM_IT_CC3)																//TIM_GetITStatus(TIM3,TIM_IT_CC3) != RESET)		//I�����벶���ж�
	{
		
	}
	
	TIM_ClearITPendingBit(TIM3,TIM_IT_CC3|TIM_IT_Update);									//����ж�
}




	/*�������򲶻��ж�-----------------------------------------------------------
	��������			��TIM4_IRQHandler(void)
	��������			��null
	��������			������Hall�������źŽ��л���
	----------------------------------------------------------------------------*/
void	TIM4_IRQHandler(void)
{
	if(TIM4->SR & TIM_IT_CC1)																//TIM_GetITStatus(TIM4,TIM_IT_CC1)!=RESET)
	{
		if(m_motor_ctrl.m_sys_state.u8_use_svpwm	==	NOT_USE_FOC)						//ʹ�����λ���
			Hall_Runtime_Convert();															//��������
		else																				//���ʹ��FOC
		{
			m_motor_rt_para.m_reverse.u8_hall_state		=	Hall_State_Read();				//��¼hall״̬
			RotorCorrection(m_motor_rt_para.m_reverse.u8_hall_state);						//FOC
		}
			
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC1);
	}
}




	/*TIM2ʱ��Դ����ʱ��������ֵ����������---------------------------------------
	��������			��	TIM8_UP_TIM13_IRQHandler(void)
	��������			��	null
	��������			��	TIM8��ʱ����жϣ����ڴ���TIM2���Ҵ���SPI_DMA��ȡ
							�ж�Ƶ��Ϊ10kHz��
	----------------------------------------------------------------------------*/
void	TIM8_UP_TIM13_IRQHandler(void)
{
	if(TIM8->SR & TIM_IT_Update)														
	{
		tim8_cnts++;
		if(tim8_cnts == 8)																//������8������SSI��ȡ����ֵ���������ݣ�����ʱ����Ϊ0.2ms
		{
			if(m_motor_ctrl.m_sys_state.u8_abs_encoder_used)
				Requir_AbsEncoder();													//��ȡ����ֵ����������
		}
		TIM_ClearITPendingBit(TIM8,TIM_IT_Update);
	}
}



	/*ADC1 DMA������ɣ�������ɼ�------------------------------------------------
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
	uint16_t	temp_sum;
	
	if(DMA2->LISR & DMA_IT_TCIF0)																																	
	{
		m_motor_rt_para.m_current_sensor.adc_dma_shadow[0]	=	m_motor_rt_para.m_current_sensor.ADC_DMA_buf[0];
		m_motor_rt_para.m_current_sensor.adc_dma_shadow[1]	=	m_motor_rt_para.m_current_sensor.ADC_DMA_buf[1];
		m_motor_rt_para.m_current_sensor.adc_dma_shadow[2]	=	m_motor_rt_para.m_current_sensor.ADC_DMA_buf[2];
		
		if(m_motor_rt_para.m_current_sensor.u8_curr_bias_ready >= 16)
		{
			Current_Average_X8_Filter(&(m_motor_rt_para.m_current_sensor));																		//��ԭʼ�������ݽ��л��������˲����Լ�ͻ����޳�
			if(m_motor_ctrl.m_sys_state.u8_use_svpwm	==	NOT_USE_FOC)
				m_motor_rt_para.m_current_sensor.f_adc_UVW_I					=	(float)(m_motor_rt_para.m_current_sensor.i16_uvw_current)*0.0100708f;
			else
			{
				m_motor_rt_para.m_current_sensor.f_adc_U_I						=	(float)(m_motor_rt_para.m_current_sensor.i16_u_current)*0.0100708f;
				m_motor_rt_para.m_current_sensor.f_adc_V_I						=	(float)(m_motor_rt_para.m_current_sensor.i16_v_current)*0.0100708f;
				m_motor_rt_para.m_current_sensor.f_adc_W_I						=	(float)(m_motor_rt_para.m_current_sensor.i16_w_current)*0.0100708f;
			}
		}
		else
		{
			temp_sum			=	m_motor_rt_para.m_current_sensor.ADC_DMA_buf[0] + m_motor_rt_para.m_current_sensor.ADC_DMA_buf[1] + m_motor_rt_para.m_current_sensor.ADC_DMA_buf[2];
			m_motor_rt_para.m_current_sensor.i16_uvw_curr_bias					+=	(temp_sum /3);
			m_motor_rt_para.m_current_sensor.u8_curr_bias_ready++;
			if(m_motor_rt_para.m_current_sensor.u8_curr_bias_ready == 16)
				m_motor_rt_para.m_current_sensor.i16_uvw_curr_bias				=	m_motor_rt_para.m_current_sensor.i16_uvw_curr_bias>>4;		//ȡ��ֵ��Ϊƫ��
		}
		m_motor_ctrl.m_motion_ctrl.u8_current_read_data_refreshed				=	1;															//��ȡ�����������ݸ���
		DMA2->LIFCR 	=	(uint32_t)(DMA_IT_TCIF0 & 0x0F7D0F7D);																				//DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
	}
}



	/*ADC2 DMA������ɣ����ش������ɼ�------------------------------------------------
	��������			��DMA2_Stream3_IRQHandler(void)
	��������			��null
	��������			��	ADC-DMA��������жϣ���PWM�ߵ͵�ƽ�м�λ�ÿ�ʼ����������6us������
							
	----------------------------------------------------------------------------*/
void	DMA2_Stream3_IRQHandler(void)
{
	if(DMA2->LISR & DMA_IT_TCIF3)																									
	{
		m_motor_rt_para.m_torque_sensor.f_torque_A						=	(float)(m_motor_rt_para.m_torque_sensor.torque_adc_buf[0]);
		m_motor_rt_para.m_torque_sensor.f_torque_B						=	(float)(m_motor_rt_para.m_torque_sensor.torque_adc_buf[1]);

		DMA2->LIFCR 	=	(uint32_t)(DMA_IT_TCIF3 & 0x0F7D0F7D);												
	}
}	




	/*CAN�����ж�----------------------------------------------------------------
	*			����˵����	CAN1_RX0_IRQHandler(void)
	*			����˵����	��
	*			ʹ�÷�Χ��	CAN1  FIFO0 �Ľ����ж�
	---------------------------------------------------------------------------*/
void	CAN1_RX0_IRQHandler(void)
{
	U16		my_data_16;
	
	OSIntEnter();
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0) == SET)							//FIFO 0 message pending Interrupt,FIFO0�����ж�
	{
		/*���CAN�����жϱ�־λ*/
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
		/*���н��մ���*/
		CAN_Receive(CAN1,CAN_FIFO0,(CanRxMsg *)&(m_can.RxMessage));			//���պ���
		/*����HeartBeat���ģ���DriverCAN������λ����*/
		if(m_can.RxMessage.StdId  == NODE_ID)								//֡Ϊ��վ��ID
		{
			/*����޸��ٶ�λ��*/
			if(m_can.RxMessage.StdId == CMD_SET_SPD_POS_CUR)
			{
				
//				m_motor_ctrl.f_feedforward_curr		=	(float)(my_data_16.u16_data);
				
			}
			/*����޸ĵ���ֵ*/
			else if(m_can.RxMessage.StdId == CMD_SET_CURR)
			{

			}
		}
		else																	//���յ�heartbeat�󣬶�heartbeat_refreshed_flag��λ
			;
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




	/*CAN�����ж�----------------------------------------------------------------
	*			����˵����	CAN1_TX_IRQHandler(void)
	*			����˵����	��
	*			ʹ�÷�Χ��	CAN1  FIFO0 �Ľ����ж�
	----------------------------------------------------------------------------*/
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



	/*SPI DMA������ɣ�SSI�źŶ���-----------------------------------------------
	��������			��DMA2_Stream2_IRQHandler
	��������			��
	��������			��SPI1 RX DMA
	----------------------------------------------------------------------------*/
void 	DMA2_Stream2_IRQHandler(void)
{
	if(DMA2->LISR & DMA_IT_TCIF2)																					//DMA_GetITStatus(DMA2_Stream2,DMA_IT_TCIF2)!=RESET)								//DMA����ж�
	{
		m_motor_rt_para.m_abs_encoder.u8_abs_data_refreshed		=	1;

		DMA_Cmd(DMA2_Stream2,DISABLE);
		DMA_Cmd(DMA2_Stream3,DISABLE);
		
		DMA2->LIFCR		=	(uint32_t)(DMA_IT_TCIF2 & 0x0F7D0F7D);													//DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);
		DMA2->LIFCR		=	(uint32_t)(DMA_IT_TCIF3 & 0x0F7D0F7D);													//DMA_ClearITPendingBit(DMA2_Stream3,DMA_IT_TCIF3);
	}
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
