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

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
uint8_t		currst;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	函数名称			：Hall_Convert(void)
	参数含义			：null
	函数功能			：读取Hall接口状态,PB6/7/8
	----------------------------------------------------------------------------*/
uint16_t	Hall_State_Read(void)
{
	uint16_t	Hall_Value;
	Hall_Value	=	(GPIOB->IDR >> 6)&0x07;
	return	Hall_Value;
}


	/*---------------------------------------------------------------------------
	函数名称			：Hall_Convert(void)
	参数含义			：null
	函数功能			：Hall换向函数
	----------------------------------------------------------------------------*/
void	Hall_Convert(void)
{
	uint16_t	Hall_Value;
	Hall_Value		=	Hall_State_Read();
	currst			=	Hall_Value;
	//根据获得的Hall状态设置tim1的PWM状态
	if(m_motor_ctrl.u8_dir == 0)											//设置电机正转
	{
		switch	(Hall_Value)
		{
			case 0x0005:
				//L1-pwm	H2-on
				TIM1_CH1_OFF_CH1N_PWM();			//1on
				TIM1_CH2_ON_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_OFF();
				break;
			case 0x0004:
				//L1-on		H3-pwm
				TIM1_CH1_OFF_CH1N_ON();				//
				TIM1_CH2_OFF_CH2N_OFF();
				TIM1_CH3_PWM_CH3N_OFF();
				break;
			case 0x0006:
				//L2-pwm	H3-on
				TIM1_CH1_OFF_CH1N_OFF();
				TIM1_CH2_OFF_CH2N_PWM();			
				TIM1_CH3_ON_CH3N_OFF();
				break;
			case 0x0002:
				//H1-pwm	L2-on
				TIM1_CH1_PWM_CH1N_OFF();			
				TIM1_CH2_OFF_CH2N_ON();
				TIM1_CH3_OFF_CH3N_OFF();
				break;
			case 0x0003:
				//H1-on		L3-pwm
				TIM1_CH1_ON_CH1N_OFF();
				TIM1_CH2_OFF_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_PWM();
				break;
			case 0x0001:
				//H2-pwm	L3-on
				TIM1_CH1_OFF_CH1N_OFF();
				TIM1_CH2_PWM_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_ON();
				break;
			default:
				break;
		}
	}
	else if(m_motor_ctrl.u8_dir == 1)										//设置电机反转
	{
		switch	(Hall_Value)
		{
			case 0x0006:
				//L3-pwm	H2-on
				TIM1_CH1_OFF_CH1N_OFF();
				TIM1_CH2_ON_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_PWM();
				break;
			case 0x0004:
				//L3-on		H1-pwm
				TIM1_CH1_PWM_CH1N_OFF();
				TIM1_CH2_OFF_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_ON();
				break;
			case 0x0005:
				//L2-pwm	H1-on
				TIM1_CH1_ON_CH1N_OFF();
				TIM1_CH2_OFF_CH2N_PWM();
				TIM1_CH3_OFF_CH3N_OFF();
				break;
			case 0x0001:
				//H3-pwm	L2-on
				TIM1_CH1_OFF_CH1N_OFF();
				TIM1_CH2_OFF_CH2N_ON();
				TIM1_CH3_PWM_CH3N_OFF();
				break;
			case 0x0003:
				//H3-on		L1-pwm
				TIM1_CH1_OFF_CH1N_PWM();
				TIM1_CH2_OFF_CH2N_OFF();
				TIM1_CH3_ON_CH3N_OFF();
				break;
			case 0x0002:
				//H2-pwm	L1-on
				TIM1_CH1_OFF_CH1N_ON();
				TIM1_CH2_PWM_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_OFF();
				break;
			default:
				break;
		}	
	}
}
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
		GPIOC->ODR	|=	0x0001;																//置高C1
		
		
//		ADC_DMACmd(ADC1,ENABLE);
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
		
//		GPIOC->ODR	&=	0xFFFE;
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
		//------------------------------------------test20180625
//		ADC_DMACmd(ADC1,DISABLE);
		m_motor_rt_para.u16_u_current	=	(m_motor_rt_para.ADC_DMA_buf[0]+m_motor_rt_para.ADC_DMA_buf[4]+m_motor_rt_para.ADC_DMA_buf[8]+m_motor_rt_para.ADC_DMA_buf[12])>>2;
		m_motor_rt_para.u16_v_current	=	(m_motor_rt_para.ADC_DMA_buf[1]+m_motor_rt_para.ADC_DMA_buf[5]+m_motor_rt_para.ADC_DMA_buf[9]+m_motor_rt_para.ADC_DMA_buf[13])>>2;
		m_motor_rt_para.u16_w_current	=	(m_motor_rt_para.ADC_DMA_buf[2]+m_motor_rt_para.ADC_DMA_buf[6]+m_motor_rt_para.ADC_DMA_buf[10]+m_motor_rt_para.ADC_DMA_buf[14])>>2;
		m_motor_rt_para.u16_power_voltage	=	(m_motor_rt_para.ADC_DMA_buf[3]+m_motor_rt_para.ADC_DMA_buf[7]+m_motor_rt_para.ADC_DMA_buf[11]+m_motor_rt_para.ADC_DMA_buf[15])>>2;
		
		m_motor_rt_para.f_adc1			=	((float)m_motor_rt_para.u16_u_current)*10.0708007812f - 15625.0f;
		m_motor_rt_para.f_adc2			=	((float)m_motor_rt_para.u16_v_current)*0.000805664f;
		m_motor_rt_para.f_adc3			=	((float)m_motor_rt_para.u16_w_current)*0.000805664f;
		m_motor_rt_para.f_adc4			=	((float)m_motor_rt_para.u16_power_voltage)*0.000805664f;
//		ADC_DMACmd(ADC1,ENABLE);
		
//		GPIOC->ODR	|=	0x0001;																//置高C1
//		ADC_DMACmd(ADC1,DISABLE);
//		GPIOC->ODR	&=	0xFFFE;																//置低C1
		//------------------------------------------test20180625
		DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
	}
}




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
