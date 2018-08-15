/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
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
#include	"spd_pos_filter.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define			RADIANS				1.047197533333f			//定义1弧度
#define			USE_CURRENT_TRACE							//使用电流模拟跟随

#ifdef			USE_CURRENT_TRACE
	#define		_PI					3.1415926535897932384626433832795f
#endif
/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
uint8_t			polarity			=	0;					//用于判断下桥臂何时打开

#ifdef	USE_CURRENT_TRACE
uint32_t		step_cnt			=	0;					//产生正弦输入
uint32_t		sim_trace_current	=	0;					//计算获得的目标电流值
float32_t		f_sin				=	0.0f;				//正弦
#endif

extern	const	uint8_t		current_senser_table[2][7];

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
	/*---------------------------------------------------------------------------
	函数名称			：TIM1_UP_TIM10_IRQHandler(void)
	参数含义			：null
	函数功能			：PWM计数器溢出中断，产生频率是40KHz，是PWM频率的2倍
	----------------------------------------------------------------------------*/
void	TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET)
	{
		polarity ++;
		if(polarity ==2)
		{		
//			GPIOC->ODR	|=	0x0001;															//置高C1
			ADC_SoftwareStartConv(ADC1);													//开启ADC采样
			polarity	=	0;
		}
		
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);											//清除TIM1中断标志
//		GPIOC->ODR	&=	0xFFFE;
	}
}


	/*---------------------------------------------------------------------------
	函数名称			：	TIM2_IRQHandler(void)
	参数含义			：	null
	函数功能			：	定时更新电流环，
							20Khz（50us周期）更新中，更新周期为占用时间2.3us(使用DAC)
							CPU使用率1.8%，1.1us（中断中没有DAC部分）
							
	----------------------------------------------------------------------------*/
void	TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
	{
		//产生目标正弦电流信号，计算时间4us
#ifdef	USE_CURRENT_TRACE
		f_sin	=	1.0f + 0.5f * arm_sin_f32( (float32_t)step_cnt * _PI / 1000.0f );
		step_cnt++;
		if(step_cnt >= 2000)
			step_cnt = 0;
		m_current_pid.curr_pid.Ref_In	=	0.5f;//f_sin;
#endif
//		m_current_pid.curr_pid.Ref_In	=	m_motor_ctrl.f_set_current;
		
		Curr_PID_Cal(&(m_current_pid.curr_pid));											//电流环更新程序部分
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);											//清除中断标志位
	}
}



	/*---------------------------------------------------------------------------
	函数名称			：	TIM3_IRQHandler(void)
	参数含义			：	null
	函数功能			：	TIM3编码器模式溢出中断，以及I相输入捕获中断							
	----------------------------------------------------------------------------*/
void	TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET)										//AB相计数溢出中断
	{
		if(TIM3->CR1 &0x0010)																//根据方向标志位
			m_motor_rt_para.i64_pulse_cnt -= 65535;
		else
			m_motor_rt_para.i64_pulse_cnt += 65535;
	}
	
	if(TIM_GetITStatus(TIM3,TIM_IT_CC3) != RESET)											//I相输入捕获中断
	{
		
	}
	
	TIM_ClearITPendingBit(TIM3,TIM_IT_CC3|TIM_IT_Update);									//清除中断
}




	/*---------------------------------------------------------------------------
	函数名称			：TIM4_IRQHandler(void)
	参数含义			：null
	函数功能			：捕获Hall传感器信号进行换向
	----------------------------------------------------------------------------*/
void	TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_CC1)!=RESET)
	{
		Hall_Convert();																		//霍尔换向
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC1);
	}
}


	/*---------------------------------------------------------------------------
	函数名称			：	TIM9_IRQHandler(void)
	参数含义			：	null
	函数功能			：	速度环以及位置环更新			
							更新频率为1KHz
	----------------------------------------------------------------------------*/
void	TIM1_BRK_TIM9_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM9,TIM_IT_Update) != RESET)	
	{
		GPIOC->ODR	|=	0x0001;
		Read_IncEncoder();
//		Speed_PID_Cal(&(m_speed_pid.spd_pid));
		GPIOC->ODR	&=	0xFFFE;
	}
	TIM_ClearITPendingBit(TIM9,TIM_IT_Update);	
}




	/*---------------------------------------------------------------------------
	函数名称			：DMA2_Stream0_IRQHandler(void)
	参数含义			：null
	函数功能			：ADC-DMA传输完成中断，在PWM高低电平中间位置开始采样，持续7us结束，

							对于BLDC的电流，硬件电路采用每个下桥臂一个采样电阻，当下桥臂导通时
							当前电阻采样的电流值为电机相电流值，故采样电流和电机当前相位有关，即
							与当前霍尔状态有关

	----------------------------------------------------------------------------*/
void	DMA2_Stream0_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0))
	{
		//电源电压采用分压，分压系数为10/92 = 0.108695，测量值Uraw与实际电压U关系为
		//		U = Uraw * 3.3 / 4096 / 0.108695 
		//
		//	
		Current_Average_X4_Filter(&m_motor_rt_para);												//对原始电流数据进行滑动窗口滤波，以及突变点剔除
		
		m_motor_rt_para.f_adc_UVW_I[0]		=	((float)(m_motor_rt_para.u16_uvw_current[0] - m_motor_rt_para.u16_uvw_curr_bias[0]))*0.0100708f;
		m_motor_rt_para.f_adc_UVW_I[1]		=	((float)(m_motor_rt_para.u16_uvw_current[1] - m_motor_rt_para.u16_uvw_curr_bias[1]))*0.0100708f;
		m_motor_rt_para.f_adc_UVW_I[2]		=	((float)(m_motor_rt_para.u16_uvw_current[2] - m_motor_rt_para.u16_uvw_curr_bias[2]))*0.0100708f;

		DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
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
