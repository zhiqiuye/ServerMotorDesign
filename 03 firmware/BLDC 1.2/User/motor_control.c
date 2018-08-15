/**
  ******************************************************************************
  * @file    Project/user/motor_control.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170728
  * @brief   ���������غ�������PWM�Ĳ���������PID���㣬��������ȡ��
  ******************************************************************************
  ******************************************************************************
  */

#include	"stm32f4xx.h"
#include	"stm32f4xx_gpio.h"
#include	"stm32f4xx_tim.h"
#include	"motor_control.h"
#include	"global_parameters.h"
#include	"peripherial_init.h"
#include	"stm32f4xx_dac.h"
#include	"ucos_ii.h"
#include	"current_filter.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	��������			��Increment_PID_Cal(PID_Struct* pid)
	��������			��null
	��������			������PID������̣����ֵ��-10��10֮��
							���غ���
	----------------------------------------------------------------------------*/
float	Increment_PID_Cal(PID_Struct* pid,float new_feedback)
{
	pid->Feed_Back			=	new_feedback;													//���·���ֵ
	pid->Err_T_2			=	pid->Err_T_1;													//����T-2ʱ�����
	pid->Err_T_1			=	pid->Err_T_0;													//����T-1ʱ�����
	pid->Err_T_0			=	pid->Ref_In - pid->Feed_Back;									//�����µ����
	
	pid->P_Out				=	pid->Kp * (pid->Err_T_0 - pid->Err_T_1);						//��������
	pid->I_Out				=	pid->Ki * pid->Err_T_0;											//���ֲ���
//	pid->D_Out				=	pid->Kd * (pid->Err_T_0 - 2.0f * pid->Err_T_1 + pid->Err_T_2);	//΢�ֲ���
	
	pid->pid_increment		=	pid->P_Out + pid->I_Out;// + pid->D_Out;							//����PID����
	
	pid->Out_Actual			=	pid->Out_Pre + pid->pid_increment;
	
	if(pid->Out_Actual > 10.0f)
		pid->Out_Actual		=	10.0f;
	if(pid->Out_Actual < -10.0f)
		pid->Out_Actual		=	-10.0f;
	
	pid->Out_Pre			=	pid->Out_Actual;
	
	return	pid->Out_Actual;
}

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
	uint16_t			Hall_Value;
	Hall_Value		=	Hall_State_Read();
	//���ݻ�õ�Hall״̬����tim1��PWM״̬
	if(m_motor_ctrl.u8_dir == 0)											//���õ����ת
	{
		switch	(Hall_Value)
		{
			case 0x0005:
				//L1-pwm	H2-on
				TIM1_CH1_OFF_CH1N_PWM();
				TIM1_CH2_PWM_CH2N_OFF();
				TIM1_CH3_OFF_CH3N_OFF();
				break;
			case 0x0004:
				//L1-on		H3-pwm
				TIM1_CH1_OFF_CH1N_PWM();
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





	/*---------------------------------------------------------------------------
	��������			��PWM_TIM_Start(void)
	��������			��null
	��������			��������ʱ������PWM
	----------------------------------------------------------------------------*/
void	PWM_TIM_Start(void)
{
	TIM_Cmd(TIM1,ENABLE);															//ʹ�ܶ�ʱ��1
}




	/*---------------------------------------------------------------------------
	��������			��PWM_TIM_Halt(void)
	��������			��null
	��������			����ͣ��ʱ������PWM
	----------------------------------------------------------------------------*/
void	PWM_TIM_Halt(void)
{
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
	��������			��SpeedPosLoopRefresh_TIM_Start(void)
	��������			��null
	��������			�������ٶ�λ�û��ĸ��¶�ʱ���ж�
	----------------------------------------------------------------------------*/
void	SpeedPosLoopRefresh_TIM_Start(void)
{
	TIM_ClearFlag(TIM9,TIM_IT_Update);
	TIM_Cmd(TIM9,ENABLE);
}



	/*---------------------------------------------------------------------------
	��������			��SpeedPosLoopRefresh_TIM_Halt(void)
	��������			��null
	��������			�������ٶ�λ�û��ĸ��¶�ʱ���ж�
	----------------------------------------------------------------------------*/
void	SpeedPosLoopRefresh_TIM_Halt(void)
{
	TIM_Cmd(TIM9,DISABLE);
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



	/*---------------------------------------------------------------------------
	��������			��Read_Current_Bias(void)
	��������			��null
	��������			����ȡ��������ֵƫ��
	----------------------------------------------------------------------------*/
void	Read_Current_Bias(void)
{
	uint8_t	i;
	
	if(m_sys_state.u8_cur_state != Prepare_state)						//����״̬��״̬
		m_sys_state.u8_cur_state = Prepare_state;
	
	CurrentLoopRefresh_TIM_Halt();										//��ͣ����������
	
	TIM1->CCR1		=	10;												//��PWMռ�ձ���Ϊ���
	TIM1->CCR2		=	10;
	TIM1->CCR3		=	10;
	
	IR2130S_force_reset();												//ռ�ձ�Ϊ0
	
	Current_Filter_Init();												//��ʼ�������˲���
	
	for(i = 0 ; i < 8 ; i++)											//��ʵ�ʵ���Ϊ0��ʱ��ÿ10msȡһ�ε���ֵѹ��buf
	{
		m_motor_rt_para.u16_u_curr_bias	+=	m_motor_rt_para.u16_u_current;
		m_motor_rt_para.u16_v_curr_bias	+=	m_motor_rt_para.u16_v_current;
		m_motor_rt_para.u16_w_curr_bias	+=	m_motor_rt_para.u16_w_current;
		OSTimeDlyHMSM(0,0,0,20);
	}

	m_motor_rt_para.u16_u_curr_bias		=	m_motor_rt_para.u16_u_curr_bias>>3;
	m_motor_rt_para.u16_v_curr_bias		=	m_motor_rt_para.u16_v_curr_bias>>3;
	m_motor_rt_para.u16_w_curr_bias		=	m_motor_rt_para.u16_w_curr_bias>>3;
	
	m_motor_rt_para.f_adc_U_I			=	0.0f;						//��������µĵ���ֵ���㣬����Ӱ���һ�ε�������PID����
	m_motor_rt_para.f_adc_V_I			=	0.0f;
	m_motor_rt_para.f_adc_W_I			=	0.0f;
}


	/*---------------------------------------------------------------------------
	��������			��Curr_PID_Cal(volatile PID_Struct * pid)
	��������			��null
	��������			��������PID���㣬����������ֻ����ֵ��������PWMռ�ձȵ�����
							����ʽPID����
	----------------------------------------------------------------------------*/
void	Curr_PID_Cal(volatile PID_Struct * pid)
{
	uint32_t		ccr;
	float			f_temp;
	uint32_t		u32_temp;
	float			pid_inc = 0.0f;
	float			curr_in = 0.0f;
	
	/*�ж��������ֵ�Ƿ��ڶ������*/
	curr_in					=	m_motor_rt_para.f_adc_U_I;
	
	if(m_motor_rt_para.f_adc_U_I > m_motor.f_max_current)
		curr_in	 			=	m_motor.f_max_current;
	
	/*��������pid���*/
	pid_inc					=	Increment_PID_Cal((PID_Struct*)pid,curr_in);			//��ʱʹ��PI
	
	//------------test 20180808	
	//------------20180813  ��E���
	f_temp					=	m_current_pid.curr_pid.Feed_Back;							//����Ŀ���ѹ
	u32_temp				=	(uint32_t)(f_temp * 1240.9f);
	
	if(u32_temp>4095) u32_temp = 4095;
	
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);
	
	//------------test 20180808
	
	/*��pid���ֵת����10-4190֮��*/
	m_current_pid.PW		+=	pid_inc;
	
	/*��PID���������ֵ����*/
	if(m_current_pid.PW > MAX_DUTY_CYCLE)
		m_current_pid.PW		=	MAX_DUTY_CYCLE;
	else if(m_current_pid.PW < MIN_DUTY_CYCLE)
		m_current_pid.PW		=	MIN_DUTY_CYCLE;
	else
		;
	
	/*����PWMռ�ձ�*/
	ccr						=	(int32_t)m_current_pid.PW;
	TIM1->CCR1				=	ccr;
	TIM1->CCR2				=	ccr;
	TIM1->CCR3				=	ccr;
	
//	ccr						=	(uint16_t)(m_motor_rt_para.f_adc_U_I * 1000.0f);

}




	/*---------------------------------------------------------------------------
	��������			��(void)
	��������			��null
	��������			��PID����
	----------------------------------------------------------------------------*/





	/*---------------------------------------------------------------------------
	��������			��(void)
	��������			��null
	��������			��PID����
	----------------------------------------------------------------------------*/
	
	
	
	
	
	/*---------------------------------------------------------------------------
	��������			��(void)
	��������			��null
	��������			��PID����
	----------------------------------------------------------------------------*/





/*********************************************END OF FILE**************************************************/

