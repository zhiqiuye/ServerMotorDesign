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
#include	"math.h"										//浮点计算使用头文件
#include	"arm_math.h"
#include	"jingle_math.h"
#include	"spd_pos_filter.h"
#include	"hall_reversal_6steps.h"
#include	"hall_reversal_svpwm.h"
#include	"peripherial_init.h"
#include	"node_can_config.h"




/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define			USE_CURRENT_TRACE							//使用电流模拟跟随
//#define			USE_SPEED_TRACE								//使用速度环模拟跟随
//#define			USE_POSITION_TRACE							//使用位置环模拟跟随

/* Private macro -------------------------------------------------------------*/
#define			RADIANS				1.047197533333f			//定义1弧度
#define			_2PI				6.283185307179586476925286766559f
#define			_PI					3.1415926535897932384626433832795f
#define			_0_5PI				1.5707963267948966192313216916398f

/* Private variables ---------------------------------------------------------*/
#ifdef	USE_CURRENT_TRACE
uint32_t		step_cnt			=	0;					//产生正弦输入
uint32_t		sim_trace_current	=	0;					//计算获得的目标电流值
float32_t		f_sin				=	0.0f;				//正弦
#define			CURRENT_AMP			1.0f
#define			CURRENT_FQC			1.0f
#define			CURRENT_CYC			(20000.0f/CURRENT_FQC)
#endif

#ifdef	USE_SPEED_TRACE
uint32_t		step_cnt			=	0;					//产生正弦输入
uint32_t		sim_trace_current	=	0;					//计算获得的目标电流值
float32_t		f_sin				=	0.0f;				//正弦
#define			SPEED_AMP			1.5f
#define			SPEED_FQC			25.0f
#define			SPEED_CYC			(1000.0f/SPEED_FQC)
#endif

#ifdef	USE_POSITION_TRACE
uint32_t		step_cnt			=	0;					//产生正弦输入
uint32_t		sim_trace_current	=	0;					//计算获得的目标电流值
float32_t		f_sin				=	0.0f;				//正弦
#define			POS_AMP				2.0f
#define			POS_FQC				1.0f
#define			POS_CYC				(1000.0f/POS_FQC)
#endif

uint8_t			tim8_cnts = 0;


/* Private function prototypes -----------------------------------------------*/
//			GPIOC->ODR	|=	0x0001;				//置高C1
//			GPIOC->ODR	&=	0xFFFE;		
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
	/*电流环/脉宽更新------------------------------------------------------------
	函数名称			：TIM1_UP_TIM10_IRQHandler(void)
	参数含义			：null
	函数功能			：	PWM计数器溢出中断，产生频率是40KHz，是PWM频率的2倍
							在PWM计数器上升溢出中断中打开ADC采样，采样耗时7us;
							
							并且更新电流环:
							20Khz（50us周期）更新中，更新时间1.5us							
	----------------------------------------------------------------------------*/
void	TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM1->SR & TIM_IT_Update)																		//TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET)
	{
		/*当计数器向上溢出中断时进行ADC采集*/
		if(TIM1->CR1 & 0x0010)
		{
			/*如果开启电流环更新，不做电流跟随的情况下，下面代码运行时间1.3us*/
			if(m_motor_ctrl.m_motion_ctrl.u8_is_currloop_used ==	1)
			{
#ifdef	USE_CURRENT_TRACE	/*产生目标正弦电流信号，计算时间4us*/
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
				/*电流值更新后，进入电流环的DMA中断*/
				if(m_motor_ctrl.m_motion_ctrl.u8_current_read_data_refreshed	==	1)
				{
					if(m_motor_ctrl.m_sys_state.u8_use_svpwm	==	NOT_USE_FOC)						//使用六步梯形换向
						Current_PID_Cal(&(m_pid.curr));													//有刷新反馈电流值，电流环更新程序部分
					else																				//使用FOC
						FOC_Cal();
					m_motor_ctrl.m_motion_ctrl.u8_current_read_data_refreshed	=	0;									
				}
			}
		}
		else
		{
			/*开启ADC DMA*/
			ADC_SoftwareStartConv(ADC1);																//开启ADC采样，相电流采集
			ADC_SoftwareStartConv(ADC2);																//开启ADC采样，力矩传感器
			
			/*速度环数据更新后，更新目标电流*/
			if(m_motor_ctrl.m_motion_ctrl.u8_current_set_data_refreshed			== 1)
			{
				if(m_motor_ctrl.m_sys_state.u8_use_svpwm	==	NOT_USE_FOC)							//使用六步梯形换向
					m_pid.curr.Ref_In											=	(float)fabs((double)m_motor_ctrl.m_motion_ctrl.f_set_current);	//电流值取绝对值
				else																					//使用FOC
				{
					m_pid.iq.Ref_In												=	0.6f;
					m_pid.id.Ref_In												=	0.0f;
				}
					
				m_motor_ctrl.m_motion_ctrl.u8_current_set_data_refreshed		=	0;
			}
		}
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);														//清除TIM1中断标志
	}
}


	/*速度环/位置环更新----------------------------------------------------------
	函数名称			：	TIM2_IRQHandler(void)
	参数含义			：	null
	函数功能			：	定时更新速度位置环，1Khz（1ms周期）更新中，
							TIM2的时钟由TIM8产生
							速度环更新耗时	3.5us
	----------------------------------------------------------------------------*/
void	TIM2_IRQHandler(void)
{
	if(TIM2->SR & TIM_IT_Update)														//TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
	{
		tim8_cnts	=	0;
		Read_IncEncoder();																//读取编码器数据，耗时1us
		if(m_motor_ctrl.m_sys_state.u8_abs_encoder_used)
			Read_AbsEncoder();															//解析SSI读取数据
		
		/*更新位置环*/
		if(m_motor_ctrl.m_motion_ctrl.u8_is_posloop_used	==	1)
		{
#ifdef	USE_POSITION_TRACE	/*产生目标正弦位置信号，*/
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
		
		/*更新速度环*/
		if(m_motor_ctrl.m_motion_ctrl.u8_is_speedloop_used	==	1)
		{
			if(m_motor_ctrl.m_motion_ctrl.u8_speed_read_data_refreshed	==	1)						//更新速度环pid参数
			{	
#ifdef	USE_SPEED_TRACE	/*产生目标正弦速度信号，计算时间4us*/
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
				m_motor_ctrl.m_motion_ctrl.u8_speed_read_data_refreshed		=	0;						//使用完最新速度值后，将标志位置低
			}				
			if(m_motor_ctrl.m_motion_ctrl.u8_speed_set_data_refreshed		==	1)						//更新参考速度值
			{
				m_pid.spd.Ref_In											=	m_motor_ctrl.m_motion_ctrl.f_set_speed;	//将设置速度值值写入PID参数
				m_motor_ctrl.m_motion_ctrl.u8_speed_set_data_refreshed		=	0;
				m_motor_ctrl.m_motion_ctrl.u8_current_set_data_refreshed	=	1;						//速度环更新标志位
			}
		}
		
		CAN_SEND_CVP();																					//can发送电流速度位置值
		
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);														//清除中断标志位
	}
}



	/*增量编码器溢出-------------------------------------------------------------
	函数名称			：	TIM3_IRQHandler(void)
	参数含义			：	null
	函数功能			：	TIM3编码器模式溢出中断，以及I相输入捕获中断							
	----------------------------------------------------------------------------*/
void	TIM3_IRQHandler(void)
{
	if(TIM3->SR & TIM_IT_Update)															//TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET)		//AB相计数溢出中断
	{
	}
	
	if(TIM3->SR & TIM_IT_CC3)																//TIM_GetITStatus(TIM3,TIM_IT_CC3) != RESET)		//I相输入捕获中断
	{
		
	}
	
	TIM_ClearITPendingBit(TIM3,TIM_IT_CC3|TIM_IT_Update);									//清除中断
}




	/*霍尔换向捕获中断-----------------------------------------------------------
	函数名称			：TIM4_IRQHandler(void)
	参数含义			：null
	函数功能			：捕获Hall传感器信号进行换向
	----------------------------------------------------------------------------*/
void	TIM4_IRQHandler(void)
{
	if(TIM4->SR & TIM_IT_CC1)																//TIM_GetITStatus(TIM4,TIM_IT_CC1)!=RESET)
	{
		if(m_motor_ctrl.m_sys_state.u8_use_svpwm	==	NOT_USE_FOC)						//使用梯形换向
			Hall_Runtime_Convert();															//霍尔换向
		else																				//如果使用FOC
		{
			m_motor_rt_para.m_reverse.u8_hall_state		=	Hall_State_Read();				//记录hall状态
			RotorCorrection(m_motor_rt_para.m_reverse.u8_hall_state);						//FOC
		}
			
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC1);
	}
}




	/*TIM2时钟源，定时触发绝对值编码器读数---------------------------------------
	函数名称			：	TIM8_UP_TIM13_IRQHandler(void)
	参数含义			：	null
	函数功能			：	TIM8定时溢出中断，用于触发TIM2并且触发SPI_DMA读取
							中断频率为10kHz，
	----------------------------------------------------------------------------*/
void	TIM8_UP_TIM13_IRQHandler(void)
{
	if(TIM8->SR & TIM_IT_Update)														
	{
		tim8_cnts++;
		if(tim8_cnts == 8)																//计数到8，启动SSI读取绝对值编码器数据，读数时间大概为0.2ms
		{
			if(m_motor_ctrl.m_sys_state.u8_abs_encoder_used)
				Requir_AbsEncoder();													//读取绝对值编码器数据
		}
		TIM_ClearITPendingBit(TIM8,TIM_IT_Update);
	}
}



	/*ADC1 DMA传输完成，相电流采集------------------------------------------------
	函数名称			：DMA2_Stream0_IRQHandler(void)
	参数含义			：null
	函数功能			：	ADC-DMA传输完成中断，在PWM高低电平中间位置开始采样，持续6us结束，
							4均值滤波计算时间为1.25us；

							对于BLDC的电流，硬件电路采用每个下桥臂一个采样电阻，
							当上桥臂改变通路时，对应的下桥臂由于续流，对应下桥臂的电流不会突变，
							使当前下桥臂仍然有电流。
							所以实际bldc的电流值为三路电流绝对值的和除以2

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
			Current_Average_X8_Filter(&(m_motor_rt_para.m_current_sensor));																		//对原始电流数据进行滑动窗口滤波，以及突变点剔除
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
				m_motor_rt_para.m_current_sensor.i16_uvw_curr_bias				=	m_motor_rt_para.m_current_sensor.i16_uvw_curr_bias>>4;		//取均值作为偏置
		}
		m_motor_ctrl.m_motion_ctrl.u8_current_read_data_refreshed				=	1;															//读取电流反馈数据更新
		DMA2->LIFCR 	=	(uint32_t)(DMA_IT_TCIF0 & 0x0F7D0F7D);																				//DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
	}
}



	/*ADC2 DMA传输完成，力矩传感器采集------------------------------------------------
	函数名称			：DMA2_Stream3_IRQHandler(void)
	参数含义			：null
	函数功能			：	ADC-DMA传输完成中断，在PWM高低电平中间位置开始采样，持续6us结束，
							
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




	/*CAN接收中断----------------------------------------------------------------
	*			函数说明：	CAN1_RX0_IRQHandler(void)
	*			参数说明：	无
	*			使用范围：	CAN1  FIFO0 的接收中断
	---------------------------------------------------------------------------*/
void	CAN1_RX0_IRQHandler(void)
{
	U16		my_data_16;
	
	OSIntEnter();
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0) == SET)							//FIFO 0 message pending Interrupt,FIFO0接收中断
	{
		/*清除CAN接收中断标志位*/
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
		/*进行接收处理*/
		CAN_Receive(CAN1,CAN_FIFO0,(CanRxMsg *)&(m_can.RxMessage));			//接收函数
		/*区分HeartBeat报文，对DriverCAN进行置位操作*/
		if(m_can.RxMessage.StdId  == NODE_ID)								//帧为本站点ID
		{
			/*如果修改速度位置*/
			if(m_can.RxMessage.StdId == CMD_SET_SPD_POS_CUR)
			{
				
//				m_motor_ctrl.f_feedforward_curr		=	(float)(my_data_16.u16_data);
				
			}
			/*如果修改电流值*/
			else if(m_can.RxMessage.StdId == CMD_SET_CURR)
			{

			}
		}
		else																	//接收到heartbeat后，对heartbeat_refreshed_flag置位
			;
	}
	if(CAN_GetITStatus(CAN1,CAN_IT_FF0) == SET)									//FIFO 0 full Interrupt,FIFO0接收满了
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_FF0);	
		/*进行FIFO清理*/
		//
		//
	}
	OSIntExit();
}




	/*CAN发送中断----------------------------------------------------------------
	*			函数说明：	CAN1_TX_IRQHandler(void)
	*			参数说明：	无
	*			使用范围：	CAN1  FIFO0 的接收中断
	----------------------------------------------------------------------------*/
void	CAN1_TX_IRQHandler(void)
{
	OSIntEnter();
	if(m_can.CAN_msg_num[0])
	{
		if(CAN1->TSR & CAN_TSR_RQCP0)									//发送邮箱0请求完成中断, RQCP0
		{
			CAN1->TSR 					=	CAN_TSR_RQCP0;				//写入1清零
			CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);
			m_can.CAN_msg_num[0]		=	0;			
		}
	}
	else if(m_can.CAN_msg_num[1])
	{
		if(CAN1->TSR & CAN_TSR_RQCP1)									//发送邮箱1请求完成中断, RQCP1
		{
			CAN1->TSR 					=	CAN_TSR_RQCP1;				//写入1清零
			CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);
			m_can.CAN_msg_num[1]		=	0;
		}
	}
	else if(m_can.CAN_msg_num[2])
	{
		if(CAN1->TSR & CAN_TSR_RQCP2)									//发送邮箱2请求完成中断, RQCP2
		{
			CAN1->TSR 					=	CAN_TSR_RQCP2;				//写入1清零
			CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);
			m_can.CAN_msg_num[2]		=	0;
		}
	}
	OSIntExit();
}



	/*SPI DMA传输完成，SSI信号读完-----------------------------------------------
	函数名称			：DMA2_Stream2_IRQHandler
	参数含义			：
	函数功能			：SPI1 RX DMA
	----------------------------------------------------------------------------*/
void 	DMA2_Stream2_IRQHandler(void)
{
	if(DMA2->LISR & DMA_IT_TCIF2)																					//DMA_GetITStatus(DMA2_Stream2,DMA_IT_TCIF2)!=RESET)								//DMA完成中断
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
