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


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

	/*---------------------------------------------------------------------------
	函数名称			：TIM1_UP_TIM10_IRQHandler(void)
	参数含义			：null
	函数功能			：PWM计数器溢出中断
	----------------------------------------------------------------------------*/
void	TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET)
	{
//		GPIOC->ODR	|=	0x0001;																//置高C1
		ADC_SoftwareStartConv(ADC1);
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
		
//		GPIOC->ODR	&=	0xFFFE;
	}
}

	/*---------------------------------------------------------------------------
	函数名称			：TIM2_IRQHandler(void)
	参数含义			：null
	函数功能			：定时更新电流环
	----------------------------------------------------------------------------*/
void	TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=RESET)
	{
		//电流环更新程序部分
		
		//清除中断标志位
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
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
		Hall_Convert();
		
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC1);
	}
}




	/*---------------------------------------------------------------------------
	函数名称			：DMA2_Stream0_IRQHandler(void)
	参数含义			：null
	函数功能			：ADC-DMA传输完成中断
	----------------------------------------------------------------------------*/
void	DMA2_Stream0_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0))
	{
		//电源电压采用分压，分压系数为10/92 = 0.108695，测量值Uraw与实际电压U关系为
		//		U = Uraw * 3.3 / 4096 / 0.108695 
		//
		//
		m_motor_rt_para.u16_power_voltage	=	(m_motor_rt_para.ADC_DMA_buf[0]+m_motor_rt_para.ADC_DMA_buf[4]+m_motor_rt_para.ADC_DMA_buf[8]+m_motor_rt_para.ADC_DMA_buf[12])>>2;
		m_motor_rt_para.u16_u_current		=	(m_motor_rt_para.ADC_DMA_buf[1]+m_motor_rt_para.ADC_DMA_buf[5]+m_motor_rt_para.ADC_DMA_buf[9]+m_motor_rt_para.ADC_DMA_buf[13])>>2;
		m_motor_rt_para.u16_v_current		=	(m_motor_rt_para.ADC_DMA_buf[2]+m_motor_rt_para.ADC_DMA_buf[6]+m_motor_rt_para.ADC_DMA_buf[10]+m_motor_rt_para.ADC_DMA_buf[14])>>2;
		m_motor_rt_para.u16_w_current		=	(m_motor_rt_para.ADC_DMA_buf[3]+m_motor_rt_para.ADC_DMA_buf[7]+m_motor_rt_para.ADC_DMA_buf[11]+m_motor_rt_para.ADC_DMA_buf[15])>>2;
		
		m_motor_rt_para.f_adc1			=	((float)m_motor_rt_para.u16_power_voltage)*0.0074121538f;
		m_motor_rt_para.f_adc2			=	((float)(m_motor_rt_para.u16_u_current - 1601))*0.0100708f;
		m_motor_rt_para.f_adc3			=	((float)(m_motor_rt_para.u16_v_current - 1601))*0.0100708f;
		m_motor_rt_para.f_adc4			=	((float)(m_motor_rt_para.u16_w_current - 1601))*0.0100708f;

		DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
	}
}



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
