/**
  ******************************************************************************
  * @file    Project/user/motor_control.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170728
  * @brief   电机控制相关函数，如PWM的产生，换向，编码器读取等
  ******************************************************************************
  ******************************************************************************
  */

#include	"stm32f4xx.h"
#include	"stm32f4xx_gpio.h"
#include	"stm32f4xx_tim.h"
#include	"motor_control.h"
#include	"global_parameters.h"
#include	"peripherial_init.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

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
	//根据获得的Hall状态设置tim1的PWM状态
	if(m_motor_ctrl.u8_dir == 0)											//设置电机正转
	{
		switch	(Hall_Value)
		{
			case 0x0005:
				//L1-pwm	H2-on
				TIM1_CH1_OFF_CH1N_PWM();			//1on
				TIM1_CH2_PWM_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_OFF();
				break;
			case 0x0004:
				//L1-on		H3-pwm
				TIM1_CH1_OFF_CH1N_PWM();				//
				TIM1_CH2_OFF_CH2N_OFF();
				TIM1_CH3_PWM_CH3N_OFF();
				break;
			case 0x0006:
				//L2-pwm	H3-on
				TIM1_CH1_OFF_CH1N_OFF();
				TIM1_CH2_OFF_CH2N_PWM();			
				TIM1_CH3_PWM_CH3N_OFF();
				break;
			case 0x0002:
				//H1-pwm	L2-on
				TIM1_CH1_PWM_CH1N_OFF();			
				TIM1_CH2_OFF_CH2N_PWM();
				TIM1_CH3_OFF_CH3N_OFF();
				break;
			case 0x0003:
				//H1-on		L3-pwm
				TIM1_CH1_PWM_CH1N_OFF();
				TIM1_CH2_OFF_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_PWM();
				break;
			case 0x0001:
				//H2-pwm	L3-on
				TIM1_CH1_OFF_CH1N_OFF();
				TIM1_CH2_PWM_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_PWM();
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
				TIM1_CH2_PWM_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_PWM();
				break;
			case 0x0004:
				//L3-on		H1-pwm
				TIM1_CH1_PWM_CH1N_OFF();
				TIM1_CH2_OFF_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_PWM();
				break;
			case 0x0005:
				//L2-pwm	H1-on
				TIM1_CH1_PWM_CH1N_OFF();
				TIM1_CH2_OFF_CH2N_PWM();
				TIM1_CH3_OFF_CH3N_OFF();
				break;
			case 0x0001:
				//H3-pwm	L2-on
				TIM1_CH1_OFF_CH1N_OFF();
				TIM1_CH2_OFF_CH2N_PWM();
				TIM1_CH3_PWM_CH3N_OFF();
				break;
			case 0x0003:
				//H3-on		L1-pwm
				TIM1_CH1_OFF_CH1N_PWM();
				TIM1_CH2_OFF_CH2N_OFF();
				TIM1_CH3_PWM_CH3N_OFF();
				break;
			case 0x0002:
				//H2-pwm	L1-on
				TIM1_CH1_OFF_CH1N_PWM();
				TIM1_CH2_PWM_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_OFF();
				break;
			default:
				break;
		}	
	}
}



	/*---------------------------------------------------------------------------
	函数名称			：PWM_TIM_Start(void)
	参数含义			：null
	函数功能			：开启定时器产生PWM
	----------------------------------------------------------------------------*/
void	PWM_TIM_Start(void)
{
//	TIM1->CNT	=	0;																//定时器清零
	TIM_Cmd(TIM1,ENABLE);															//使能定时器1
//	TIM_CtrlPWMOutputs(TIM1,ENABLE);												//使能timer1输出PWM
}




	/*---------------------------------------------------------------------------
	函数名称			：PWM_TIM_Halt(void)
	参数含义			：null
	函数功能			：暂停定时器产生PWM
	----------------------------------------------------------------------------*/
void	PWM_TIM_Halt(void)
{
//	TIM_CtrlPWMOutputs(TIM1,DISABLE);															//禁止timer1输出PWM
	TIM_Cmd(TIM1,DISABLE);																		//禁用定时器1
}



	/*---------------------------------------------------------------------------
	函数名称			：CoilCurrentRefresh_TIM_Start(void)
	参数含义			：null
	函数功能			：开启电流环的更新定时器中断
	----------------------------------------------------------------------------*/
void	CurrentLoopRefresh_TIM_Start(void)
{
	TIM_Cmd(TIM2,ENABLE);
}


	/*---------------------------------------------------------------------------
	函数名称			：CoilCurrentRefresh_TIM_Halt(void)
	参数含义			：null
	函数功能			：开启电流环的更新定时器中断
	----------------------------------------------------------------------------*/
void	CurrentLoopRefresh_TIM_Halt(void)
{
	TIM_Cmd(TIM2,DISABLE);
}



	/*---------------------------------------------------------------------------
	函数名称			：IR2130S_force_reset(void)
	参数含义			：null
	函数功能			：拉低L1-3，进行复位
	----------------------------------------------------------------------------*/
void	IR2130S_force_reset(void)
{
	TIM1_CH1_OFF_CH1N_PWM();
	TIM1_CH2_OFF_CH2N_PWM();
	TIM1_CH3_OFF_CH3N_PWM();
}




/*********************************************END OF FILE**************************************************/

