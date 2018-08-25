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
#include	"hall_reversal_6steps.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
//#define		MEASURE_CURRENT_REFIN
#define		MEASURE_SPEED_REFIN
//#define		MEASURE_POSITION_REFIN

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


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
	pid->D_Out				=	pid->Kd * (pid->Err_T_0 - 2.0f * pid->Err_T_1 + pid->Err_T_2);	//΢�ֲ���
	
	pid->pid_increment		=	pid->P_Out + pid->I_Out + pid->D_Out;							//����PID����
	
	pid->Out_Actual			=	pid->Out_Pre + pid->pid_increment;
	
	if(pid->Out_Actual > 10.0f)
		pid->Out_Actual		=	10.0f;
	if(pid->Out_Actual < -10.0f)
		pid->Out_Actual		=	-10.0f;
	
	pid->Out_Pre			=	pid->Out_Actual;
	
	return	pid->Out_Actual;
}

/*����ֵʹ��int�͵ļ���pid*/
float	Increment_PID_Cal_int(PID_Struct* pid, int32_t new_feedback)
{
	pid->Feed_Back			=	(float)new_feedback;													//���·���ֵ
	pid->Err_T_2			=	pid->Err_T_1;													//����T-2ʱ�����
	pid->Err_T_1			=	pid->Err_T_0;													//����T-1ʱ�����
	pid->Err_T_0			=	pid->Ref_In - pid->Feed_Back;									//�����µ����
	
	pid->P_Out				=	pid->Kp * (pid->Err_T_0 - pid->Err_T_1);						//��������
	pid->I_Out				=	pid->Ki * pid->Err_T_0;											//���ֲ���
	pid->D_Out				=	pid->Kd * (pid->Err_T_0 - 2.0f * pid->Err_T_1 + pid->Err_T_2);	//΢�ֲ���
	
	pid->pid_increment		=	pid->P_Out + pid->I_Out + pid->D_Out;							//����PID����
	
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
	��������			����ʼ����ʱHall������
	----------------------------------------------------------------------------*/
void	Hall_Start_Convert(void)
{
	m_motor_rt_para.u8_hall_state		=	Hall_State_Read();								//��¼hall״̬
	
	startup_switch_table[m_motor_ctrl.u8_dir][m_motor_rt_para.u8_hall_state]();						//���ݷ����Լ�hall��λ���л��򣬲��ú���ָ�뷽ʽ
}


	/*---------------------------------------------------------------------------
	��������			��Hall_Runtime_Convert(void)
	��������			��null
	��������			������ʱHall������
	----------------------------------------------------------------------------*/
void	Hall_Runtime_Convert(void)
{
	m_motor_rt_para.u8_hall_state		=	Hall_State_Read();								//��¼hall״̬
	
	runtime_switch_table[m_motor_ctrl.u8_dir][m_motor_rt_para.u8_hall_state]();						//���ݷ����Լ�hall��λ���л��򣬲��ú���ָ�뷽ʽ
}


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
	m_motor_ctrl.u8_is_currloop_used	=	1;
	TIM_Cmd(TIM1,ENABLE);
}


	/*---------------------------------------------------------------------------
	��������			��CoilCurrentRefresh_TIM_Halt(void)
	��������			��null
	��������			���رյ������ĸ��¶�ʱ���ж�
	----------------------------------------------------------------------------*/
void	CurrentLoopRefresh_TIM_Halt(void)
{
	m_motor_ctrl.u8_is_currloop_used	=	0;
	TIM1->CCR1	=	10;
	TIM1->CCR2	=	10;
	TIM1->CCR3	=	10;
}

	/*---------------------------------------------------------------------------
	��������			��SpeedPosLoopRefresh_TIM_Start(void)
	��������			��null
	��������			��	�����ٶȻ��ĸ��¶�ʱ���ж�
							�����ٶȻ����뿪��������
	----------------------------------------------------------------------------*/
void	SpeedLoopRefresh_TIM_Start(void)
{
	m_motor_ctrl.u8_is_speedloop_used	=	1;
	m_motor_ctrl.u8_is_currloop_used	=	1;
	
	TIM_ClearFlag(TIM2,TIM_IT_Update);
	TIM_Cmd(TIM2,ENABLE);
}



	/*---------------------------------------------------------------------------
	��������			��SpeedPosLoopRefresh_TIM_Halt(void)
	��������			��null
	��������			�������ٶȻ��ĸ��¶�ʱ���ж�
	----------------------------------------------------------------------------*/
void	SpeedLoopRefresh_TIM_Halt(void)
{
	m_motor_ctrl.u8_is_speedloop_used	=	0;
}



	/*---------------------------------------------------------------------------
	��������			��PositionLoopRefresh_TIM_Start(void)
	��������			��null
	��������			������λ�û��ĸ��¶�ʱ���ж�
	----------------------------------------------------------------------------*/
void	PositionLoopRefresh_TIM_Start(void)
{
	m_motor_ctrl.u8_is_currloop_used	=	1;
	m_motor_ctrl.u8_is_speedloop_used	=	1;
	m_motor_ctrl.u8_is_posloop_used		=	1;
	
	TIM_ClearFlag(TIM2,TIM_IT_Update);
	TIM_Cmd(TIM2,ENABLE);
}



	/*---------------------------------------------------------------------------
	��������			��PositionLoopRefresh_TIM_Start(void)
	��������			��null
	��������			���ر�λ�û��ĸ���
	----------------------------------------------------------------------------*/
void	PositionLoopRefresh_TIM_Halt(void)
{
	m_motor_ctrl.u8_is_posloop_used		=	0;
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
	
	m_motor_rt_para.u16_uvw_curr_bias		=	0x641;					//1.29V��Ϊƫ��

	m_motor_rt_para.f_adc_UVW_I				=	0.0f;					//��������µĵ���ֵ���㣬����Ӱ���һ�ε�������PID����
}


	/*---------------------------------------------------------------------------
	��������			��Curr_PID_Cal(volatile PID_Struct * pid)
	��������			��null
	��������			��������PID���㣬����������ֻ����ֵ��������PWMռ�ձȵ�����
							����ʽPID����
	----------------------------------------------------------------------------*/
void	Current_PID_Cal(volatile PID_Struct * pid)
{
	uint32_t		ccr;
	float			pid_inc	= 0.0f;
	float			curr_in = 0.0f;
	
#ifdef	MEASURE_CURRENT_REFIN
	float			f_temp;
	uint32_t		u32_temp;	
#endif
	
	/*�ж��������ֵ�Ƿ��ڶ������*/
	curr_in					=	m_motor_rt_para.f_adc_UVW_I;
	
	if(curr_in > m_motor.f_max_current)
		curr_in	 			=	m_motor.f_max_current;
	
	/*��������pid���*/
	pid_inc					=	Increment_PID_Cal((PID_Struct*)pid,curr_in);			//��ʱʹ��PI

#ifdef	MEASURE_CURRENT_REFIN	
	//------------test 20180808	
	f_temp					=	m_current_pid.curr_pid.Feed_Back;//m_current_pid.curr_pid.Ref_In;//						//����Ŀ���ѹ
	u32_temp				=	(uint32_t)(f_temp * 1240.9f);
	if(u32_temp>4095) u32_temp = 4095;													//���������޶���ֵ
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);							//����DAת��
#endif
	
	/*��pid���ֵת����10-4190֮��*/
	m_current_pid.PW		=	pid_inc * 420.0f;
	
	/*��PID���������ֵ����*/
	if(m_current_pid.PW > MAX_DUTY_CYCLE)
		m_current_pid.PW	=	MAX_DUTY_CYCLE;
	else if(m_current_pid.PW < MIN_DUTY_CYCLE)
		m_current_pid.PW	=	MIN_DUTY_CYCLE;
	else
		;
	
	/*����PWMռ�ձ�*/
	ccr						=	(uint16_t)m_current_pid.PW;
	TIM1->CCR1				=	ccr;
	TIM1->CCR2				=	ccr;
	TIM1->CCR3				=	ccr;
}




	/*---------------------------------------------------------------------------
	��������			��Speed_PID_Cal(volatile PID_Struct * pid)
	��������			���ٶȻ�pid�ṹ��
	��������			���ٶȻ���PID����
	----------------------------------------------------------------------------*/
void	Speed_PID_Cal(volatile PID_Struct * pid)
{
	float	spd_in			=	0.0f;											//��ȡ���ٶ�ֵ
	float	pid_inc			=	0.0f;
#ifdef	MEASURE_SPEED_REFIN
	uint32_t	u32_temp;
	float		f_temp;
#endif
	
	/*��������pid���*/
	spd_in					=	m_motor_rt_para.m_encoder.f_motor_cal_speed;	//��ȡ�����ٶ�ֵ
	pid_inc					=	Increment_PID_Cal((PID_Struct*)pid,spd_in*2.0f);

#ifdef	MEASURE_SPEED_REFIN
//--------------------20180817test	
	f_temp					=	m_motor_rt_para.m_encoder.f_motor_cal_speed + 1.5f; //spd_in + 1.5f;//									//����Ŀ���ѹ
	u32_temp				=	(uint32_t)(f_temp * 819.0f);//(uint32_t)(f_temp * 409.6f);//					//5rps��Ӧ3.3V
	if(u32_temp>4095) u32_temp 		= 	4095;
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);
#endif
	
	/*��pid���ֵ�ۼӵ���������ֵ*/
	m_motor_ctrl.f_set_current		=	pid_inc;		
	
	/*�޶�����ֵΪ�������Ҹ��ݵ���ֵ����������*/
	if(m_motor_ctrl.f_set_current > 0.0f)
	{
		m_motor_ctrl.u8_dir			=	1;
		startup_switch_table[m_motor_ctrl.u8_dir][m_motor_rt_para.u8_hall_state]();
	}
	else if(m_motor_ctrl.f_set_current < -0.0f)
	{
		m_motor_ctrl.u8_dir			=	0;
		startup_switch_table[m_motor_ctrl.u8_dir][m_motor_rt_para.u8_hall_state]();
	}
	else
	{
		m_motor_ctrl.f_set_current	=	0.0f;
	}
	
	m_motor_ctrl.u8_current_set_data_refreshed	=	1;							//�����������ݸ���
}

	/*---------------------------------------------------------------------------
	��������			��Position_PID_Cal(volatile PID_Struct * pid)
	��������			��λ�û�pid�ṹ��
	��������			��λ�û���PID����
	----------------------------------------------------------------------------*/
void	Position_PID_Cal(volatile PID_Struct * pid)
{
	float		pos_in				=	0.0f;
	float		pid_inc				=	0.0f;

#ifdef	MEASURE_POSITION_REFIN
	uint32_t	u32_temp;
	float		f_temp;
#endif
/*��������pid���*/
	pos_in							=	(float)m_motor_rt_para.m_encoder.i32_pulse_cnt;	//��ȡ����λ��ֵ
	pid_inc							=	Increment_PID_Cal((PID_Struct*)pid,pos_in);

#ifdef	MEASURE_POSITION_REFIN
//--------------------20180820test	
	f_temp							=	pos_in + 2000.0f;//m_motor_ctrl.f_set_position	+ 1000.0f;//					//����Ŀ���ѹ
	u32_temp						=	(uint32_t)(f_temp);						//��-1000cnts��1000cnts Ͷ�䵽0-3.3V
	if(u32_temp>4095) u32_temp 		=	4095;
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);
#endif
	
/*��pid����ֵ��ֵ������ֵ��*/
	m_motor_ctrl.f_set_speed		=	pid_inc;
	
/*�����ٶȵ��ڵ������ٶ�*/
	
	m_motor_ctrl.u8_speed_set_data_refreshed	=	1;
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





/*********************************************END OF FILE**************************************************/

