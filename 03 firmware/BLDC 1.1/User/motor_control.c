/**
  ******************************************************************************
  * @file    Project/user/motor_control.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170728
  * @brief   ���������غ�������PWM�Ĳ��������򣬱�������ȡ��
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
	��������			��Hall_Convert(void)
	��������			��null
	��������			����ȡHall�ӿ�״̬,PB6/7/8
	----------------------------------------------------------------------------*/
uint16_t	Hall_State_Read(void)
{
	uint16_t	Hall_Value;
	Hall_Value	=	(GPIOB->IDR >> 6)&0x07;
	return	Hall_Value;
}


	/*---------------------------------------------------------------------------
	��������			��Hall_Convert(void)
	��������			��null
	��������			��Hall������
	----------------------------------------------------------------------------*/
void	Hall_Convert(void)
{
	uint16_t	Hall_Value;
	Hall_Value		=	Hall_State_Read();
	//���ݻ�õ�Hall״̬����tim1��PWM״̬
	if(m_motor_ctrl.u8_dir == 0)											//���õ����ת
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
	else if(m_motor_ctrl.u8_dir == 1)										//���õ����ת
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
	��������			��PWM_TIM_Start(void)
	��������			��null
	��������			��������ʱ������PWM
	----------------------------------------------------------------------------*/
void	PWM_TIM_Start(void)
{
//	TIM1->CNT	=	0;																//��ʱ������
	TIM_Cmd(TIM1,ENABLE);															//ʹ�ܶ�ʱ��1
//	TIM_CtrlPWMOutputs(TIM1,ENABLE);												//ʹ��timer1���PWM
}




	/*---------------------------------------------------------------------------
	��������			��PWM_TIM_Halt(void)
	��������			��null
	��������			����ͣ��ʱ������PWM
	----------------------------------------------------------------------------*/
void	PWM_TIM_Halt(void)
{
//	TIM_CtrlPWMOutputs(TIM1,DISABLE);															//��ֹtimer1���PWM
	TIM_Cmd(TIM1,DISABLE);																		//���ö�ʱ��1
}



	/*---------------------------------------------------------------------------
	��������			��CoilCurrentRefresh_TIM_Start(void)
	��������			��null
	��������			�������������ĸ��¶�ʱ���ж�
	----------------------------------------------------------------------------*/
void	CurrentLoopRefresh_TIM_Start(void)
{
	TIM_Cmd(TIM2,ENABLE);
}


	/*---------------------------------------------------------------------------
	��������			��CoilCurrentRefresh_TIM_Halt(void)
	��������			��null
	��������			�������������ĸ��¶�ʱ���ж�
	----------------------------------------------------------------------------*/
void	CurrentLoopRefresh_TIM_Halt(void)
{
	TIM_Cmd(TIM2,DISABLE);
}



	/*---------------------------------------------------------------------------
	��������			��IR2130S_force_reset(void)
	��������			��null
	��������			������L1-3�����и�λ
	----------------------------------------------------------------------------*/
void	IR2130S_force_reset(void)
{
	TIM1_CH1_OFF_CH1N_PWM();
	TIM1_CH2_OFF_CH2N_PWM();
	TIM1_CH3_OFF_CH3N_PWM();
}




/*********************************************END OF FILE**************************************************/

