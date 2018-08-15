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

/* Private function prototypes -----------------------------------------------*/
const void		NULL_Switch(void);
const void		H1_L2(void);
const void		H1_L3(void);
const void		H2_L1(void);
const void		H2_L3(void);
const void		H3_L2(void);
const void		H3_L1(void);

/* Private variables ---------------------------------------------------------*/
//���������
const	void( *switch_table[2][7])() ={	{NULL_Switch,	H2_L3,		H1_L2,		H1_L3,		H3_L1,		H2_L1,		H3_L2},
										{NULL_Switch,	H3_L2,		H2_L1,		H3_L1,		H1_L3,		H1_L2,		H2_L3}}; 

//���������
const	uint8_t		current_senser_table[2][7]	=	{{0, 2, 1, 2, 0, 0, 1},{0, 1, 0, 0, 2, 1, 2}};										


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
	
	m_motor_rt_para.u8_hall_state		=	Hall_Value;						//��¼hall״̬
	
	switch_table[m_motor_ctrl.u8_dir][Hall_Value]();						//���ݷ����Լ�hall��λ���л��򣬲��ú���ָ�뷽ʽ
}


	/*---------------------------------------------------------------------------
	��������			��NULL_Switch(void)
	��������			��null
	��������			����
	----------------------------------------------------------------------------*/
const void	NULL_Switch(void)
{}

	/*---------------------------------------------------------------------------
	��������			��H1_L2(void)
	��������			��null
	��������			��H1��L2��ͨ
	----------------------------------------------------------------------------*/
const void	H1_L2(void)
{
	TIM1_CH1_PWM_CH1N_OFF();
	TIM1_CH2_OFF_CH2N_PWM();
	TIM1_CH3_OFF_CH3N_OFF();
}


	/*---------------------------------------------------------------------------
	��������			��H1_L3(void)
	��������			��null
	��������			��H1��L3��ͨ
	----------------------------------------------------------------------------*/
const void	H1_L3(void)
{
	TIM1_CH1_PWM_CH1N_OFF();
	TIM1_CH2_OFF_CH2N_OFF();
	TIM1_CH3_OFF_CH3N_PWM();
}


	/*---------------------------------------------------------------------------
	��������			��H2_L1(void)
	��������			��null
	��������			��H2��L1��ͨ
	----------------------------------------------------------------------------*/
const void	H2_L1(void)
{
	TIM1_CH1_OFF_CH1N_PWM();
	TIM1_CH2_PWM_CH2N_OFF();
	TIM1_CH3_OFF_CH3N_OFF();
}


	/*---------------------------------------------------------------------------
	��������			��H2_L3(void)
	��������			��null
	��������			��H2��L3��ͨ
	----------------------------------------------------------------------------*/
const void	H2_L3(void)
{
	TIM1_CH1_OFF_CH1N_OFF();
	TIM1_CH2_PWM_CH2N_OFF();
	TIM1_CH3_OFF_CH3N_PWM();
}


	/*---------------------------------------------------------------------------
	��������			��H3_L1(void)
	��������			��null
	��������			��H3��L1��ͨ
	----------------------------------------------------------------------------*/
const void	H3_L1(void)
{
	TIM1_CH1_OFF_CH1N_PWM();
	TIM1_CH2_OFF_CH2N_OFF();
	TIM1_CH3_PWM_CH3N_OFF();
}


	/*---------------------------------------------------------------------------
	��������			��H3_L2(void)
	��������			��null
	��������			��H3��L2��ͨ
	----------------------------------------------------------------------------*/
const void	H3_L2(void)
{
	TIM1_CH1_OFF_CH1N_OFF();
	TIM1_CH2_OFF_CH2N_PWM();
	TIM1_CH3_PWM_CH3N_OFF();
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
	
/*�Ըտ�ʼ�ϵ��ƫ�õ�ѹ��Ϊƫ��ֵ������ϴ����*/	
//	for(i = 0 ; i < 8 ; i++)											//��ʵ�ʵ���Ϊ0��ʱ��ÿ10msȡһ�ε���ֵѹ��buf
//	{
//		m_motor_rt_para.u16_uvw_curr_bias[0]	+=	m_motor_rt_para.u16_uvw_current[0];
//		m_motor_rt_para.u16_uvw_curr_bias[1]	+=	m_motor_rt_para.u16_uvw_current[1];
//		m_motor_rt_para.u16_uvw_curr_bias[2]	+=	m_motor_rt_para.u16_uvw_current[2];
//		OSTimeDlyHMSM(0,0,0,20);
//	}
//	m_motor_rt_para.u16_uvw_curr_bias[0]		=	m_motor_rt_para.u16_uvw_curr_bias[0]>>3;
//	m_motor_rt_para.u16_uvw_curr_bias[1]		=	m_motor_rt_para.u16_uvw_curr_bias[1]>>3;
//	m_motor_rt_para.u16_uvw_curr_bias[2]		=	m_motor_rt_para.u16_uvw_curr_bias[2]>>3;
	
	m_motor_rt_para.u16_uvw_curr_bias[0]		=	0x641;				//1.29V��Ϊƫ��
	m_motor_rt_para.u16_uvw_curr_bias[1]		=	0x641;
	m_motor_rt_para.u16_uvw_curr_bias[2]		=	0x641;
	
	
	
	m_motor_rt_para.f_adc_UVW_I[0]				=	0.0f;				//��������µĵ���ֵ���㣬����Ӱ���һ�ε�������PID����
	m_motor_rt_para.f_adc_UVW_I[1]				=	0.0f;
	m_motor_rt_para.f_adc_UVW_I[2]				=	0.0f;
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
	curr_in					=	m_motor_rt_para.f_adc_UVW_I[current_senser_table[m_motor_ctrl.u8_dir][m_motor_rt_para.u8_hall_state]];
	
	if(curr_in > m_motor.f_max_current)
		curr_in	 			=	m_motor.f_max_current;
	
	/*��������pid���*/
	pid_inc					=	Increment_PID_Cal((PID_Struct*)pid,curr_in);			//��ʱʹ��PI
	
	//------------test 20180808	
	//------------20180813  ��E���
	f_temp					=	m_current_pid.curr_pid.Feed_Back;						//����Ŀ���ѹ
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
	float	spd_in	=	0.0f;						//��ȡ���ٶ�ֵ
	float	pid_inc	=	0.0f;
	
	/*��������pid���*/
	spd_in		=	m_motor_rt_para.f_motor_cal_speed;
	pid_inc		=	Increment_PID_Cal((PID_Struct*)pid,spd_in);
	
	/*��pid���ֵ�ۼӵ���������ֵ*/
	m_motor_ctrl.f_set_current		+=	pid_inc * 0.1f;		
	
	/*�޶�����ֵΪ�������Ҹ��ݵ���ֵ����������*/
	if(m_motor_ctrl.f_set_current > 0.0f)
	{
		m_motor_ctrl.u8_dir			=	0;
		switch_table[m_motor_ctrl.u8_dir][m_motor_rt_para.u8_hall_state]();
	}
	else
	{
		m_motor_ctrl.u8_dir			=	1;
		m_motor_ctrl.f_set_current	=	-m_motor_ctrl.f_set_current;
		switch_table[m_motor_ctrl.u8_dir][m_motor_rt_para.u8_hall_state]();
	}
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

