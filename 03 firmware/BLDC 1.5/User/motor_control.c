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
#include	"node_can_config.h"
#include	"delay.h"
#include	"arm_math.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define		MEASURE_CURRENT_REFIN
//#define		MEASURE_SPEED_REFIN
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
float	Increment_PID_Cal(PID_struct* pid,float new_feedback)
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
float	Increment_PID_Cal_int(PID_struct* pid, int32_t new_feedback)
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
	��������			��Absolute_PID_Cal(PID_struct* pid)
	��������			��null
	��������			������ֵʽPID������̣����ֵ��-10��10֮��
							���غ���
	----------------------------------------------------------------------------*/
float	Absolute_PID_Cal(PID_struct* pid,float new_feedback)
{
	pid->Feed_Back			=	new_feedback;
	pid->Err_T_2			=	pid->Err_T_1;
	pid->Err_T_1			=	pid->Err_T_0;
	pid->Err_T_0			=	pid->Ref_In - pid->Feed_Back;
	pid->Err_Sum			+=	pid->Err_T_0;
	pid->Err_Dif			=	pid->Err_T_0 - pid->Err_T_1;
	
	pid->P_Out				=	pid->Kp * pid->Err_T_0;
	pid->I_Out				=	pid->Ki * pid->Err_Sum;
	pid->D_Out				=	pid->Kd * pid->Err_Dif;
	
	pid->Out_Actual			=	pid->P_Out + pid->I_Out + pid->D_Out;
	
	if(pid->Out_Actual > 10.0f)
		pid->Out_Actual		=	10.0f;
	if(pid->Out_Actual < -10.0f)
		pid->Out_Actual		=	-10.0f;
	
	return	pid->Out_Actual;
}


	/*---------------------------------------------------------------------------
	��������			��Hall_Convert(void)
	��������			��null
	��������			����ʼ����ʱHall������
	----------------------------------------------------------------------------*/
void	Hall_Start_Convert(void)
{
	m_motor_rt_para.m_reverse.u8_hall_state		=	Hall_State_Read();							//��¼hall״̬
	
	startup_switch_table[m_motor_ctrl.m_motion_ctrl.u8_dir][m_motor_rt_para.m_reverse.u8_hall_state]();		//���ݷ����Լ�hall��λ���л��򣬲��ú���ָ�뷽ʽ
}


	/*---------------------------------------------------------------------------
	��������			��Hall_Runtime_Convert(void)
	��������			��null
	��������			������ʱHall������
	----------------------------------------------------------------------------*/
void	Hall_Runtime_Convert(void)
{
	m_motor_rt_para.m_reverse.u8_hall_state		=	Hall_State_Read();							//��¼hall״̬
	
	runtime_switch_table[m_motor_ctrl.m_motion_ctrl.u8_dir][m_motor_rt_para.m_reverse.u8_hall_state]();		//���ݷ����Լ�hall��λ���л��򣬲��ú���ָ�뷽ʽ
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
	m_motor_ctrl.m_motion_ctrl.u8_is_currloop_used	=	1;
	TIM1->CCR1	=	10;
	TIM1->CCR2	=	10;
	TIM1->CCR3	=	10;
	TIM_Cmd(TIM1,ENABLE);
}


	/*---------------------------------------------------------------------------
	��������			��CoilCurrentRefresh_TIM_Halt(void)
	��������			��null
	��������			���رյ������ĸ��¶�ʱ���ж�
	----------------------------------------------------------------------------*/
void	CurrentLoopRefresh_TIM_Halt(void)
{
	m_motor_ctrl.m_motion_ctrl.u8_is_currloop_used	=	0;
	
	TIM1->CCR1	=	10;
	TIM1->CCR2	=	10;
	TIM1->CCR3	=	10;
	TIM_Cmd(TIM1,DISABLE);
}

	/*---------------------------------------------------------------------------
	��������			��SpeedPosLoopRefresh_TIM_Start(void)
	��������			��null
	��������			��	�����ٶȻ��ĸ��¶�ʱ���ж�
							�����ٶȻ����뿪��������
	----------------------------------------------------------------------------*/
void	SpeedLoopRefresh_TIM_Start(void)
{
	m_motor_ctrl.m_motion_ctrl.u8_is_speedloop_used	=	1;
	m_motor_ctrl.m_motion_ctrl.u8_is_currloop_used	=	1;
	
	TIM_ClearFlag(TIM2,TIM_IT_Update);
	TIM_Cmd(TIM2,ENABLE);
	TIM_Cmd(TIM8,ENABLE);
}



	/*---------------------------------------------------------------------------
	��������			��SpeedPosLoopRefresh_TIM_Halt(void)
	��������			��null
	��������			�������ٶȻ��ĸ��¶�ʱ���ж�
	----------------------------------------------------------------------------*/
void	SpeedLoopRefresh_TIM_Halt(void)
{
	m_motor_ctrl.m_motion_ctrl.u8_is_speedloop_used	=	0;
}



	/*---------------------------------------------------------------------------
	��������			��PositionLoopRefresh_TIM_Start(void)
	��������			��null
	��������			������λ�û��ĸ��¶�ʱ���ж�
	----------------------------------------------------------------------------*/
void	PositionLoopRefresh_TIM_Start(void)
{
	m_motor_ctrl.m_motion_ctrl.u8_is_currloop_used	=	1;
	m_motor_ctrl.m_motion_ctrl.u8_is_speedloop_used	=	1;
	m_motor_ctrl.m_motion_ctrl.u8_is_posloop_used		=	1;
	
	TIM_ClearFlag(TIM2,TIM_IT_Update);
	TIM_Cmd(TIM2,ENABLE);
	TIM_Cmd(TIM8,ENABLE);
}



	/*---------------------------------------------------------------------------
	��������			��PositionLoopRefresh_TIM_Start(void)
	��������			��null
	��������			���ر�λ�û��ĸ���
	----------------------------------------------------------------------------*/
void	PositionLoopRefresh_TIM_Halt(void)
{
	m_motor_ctrl.m_motion_ctrl.u8_is_posloop_used		=	0;
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
	
	if(m_motor_ctrl.m_sys_state.u8_cur_state != Prepare_state)						//����״̬��״̬
		m_motor_ctrl.m_sys_state.u8_cur_state = Prepare_state;
	
	IR2130S_force_reset();															//ռ�ձ�Ϊ0
		
	CurrentLoopRefresh_TIM_Halt();													//��ͣ����������
	
	Current_Filter_Init();															//��ʼ�������˲���

	CurrentLoopRefresh_TIM_Start();
}


	/*---------------------------------------------------------------------------
	��������			��Curr_PID_Cal(volatile PID_Struct * pid)
	��������			��null
	��������			��	������PID���㣬����������ֻ����ֵ��������PWMռ�ձȵ�����
						����ʽPID����
	----------------------------------------------------------------------------*/
void	Current_PID_Cal(volatile PID_struct * pid)
{
	uint32_t		ccr;
	float			pid_inc	= 0.0f;
	float			curr_in = 0.0f;
	
#ifdef	MEASURE_CURRENT_REFIN
	float			f_temp;
	uint32_t		u32_temp;	
	float			f_temp2;
	uint32_t		u32_temp2;
#endif
	
	/*�ж��������ֵ�Ƿ��ڶ������*/
	curr_in					=	m_motor_rt_para.m_current_sensor.f_adc_UVW_I;
	if(curr_in > m_motor_attribute_para.m_motor_att.f_max_current)
		curr_in	 			=	m_motor_attribute_para.m_motor_att.f_max_current;
	
	/*��������pid���*/
	pid_inc					=	Increment_PID_Cal((PID_struct*)pid,curr_in);			//��ʱʹ��PI

//	pid_inc					=	Absolute_PID_Cal((PID_struct*)pid,curr_in);
#ifdef	MEASURE_CURRENT_REFIN
	//��
	f_temp					=	m_pid.curr.Feed_Back;									//����Ŀ���ѹ ��
	u32_temp				=	(uint32_t)(f_temp * 1241.21f);
	if(u32_temp>4095) u32_temp = 4095;													//���������޶���ֵ
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);							//����DAת��
	
	//��
	f_temp2					=	(float) (pid_inc + 10.0f) * 200.0f;//(m_motor_rt_para.m_current_sensor.ADC_DMA_buf[2]);//
	u32_temp2				=	(uint32_t)f_temp2;
	if(u32_temp2>4095) u32_temp2 = 4095;
	DAC_SetChannel2Data(DAC_Align_12b_R,(uint16_t)u32_temp2);
#endif

	/*��pid���ֵת����10-4190֮��*/
	m_pid.PW				=	pid_inc * 420.0f;							
	
	/*�����õ���ֵ���л���*/
	if(m_pid.PW > 1.5f)
	{
		m_motor_ctrl.m_motion_ctrl.u8_dir			=	1;
	}
	else if(m_pid.PW < -1.5f)
	{
		m_motor_ctrl.m_motion_ctrl.u8_dir			=	0;
	}
	
	/*����*/
	startup_switch_table[m_motor_ctrl.m_motion_ctrl.u8_dir][m_motor_rt_para.m_reverse.u8_hall_state]();
	
	arm_abs_f32(&m_pid.PW,&m_pid.PW,1);
	
	/*��PID���������ֵ����*/
	if(m_pid.PW > MAX_DUTY_CYCLE)
		m_pid.PW	=	MAX_DUTY_CYCLE;
	else if(m_pid.PW < MIN_DUTY_CYCLE)
		m_pid.PW	=	MIN_DUTY_CYCLE;
	else
		;
	
	/*����PWMռ�ձ�*/
	ccr						=	(uint16_t)m_pid.PW;
	TIM1->CCR1				=	ccr;
	TIM1->CCR2				=	ccr;
	TIM1->CCR3				=	ccr;
}




	/*---------------------------------------------------------------------------
	��������			��Speed_PID_Cal(volatile PID_Struct * pid)
	��������			���ٶȻ�pid�ṹ��
	��������			���ٶȻ���PID����
	----------------------------------------------------------------------------*/
void	Speed_PID_Cal(volatile PID_struct * pid)
{
	float	spd_in			=	0.0f;												//��ȡ���ٶ�ֵ
	float	pid_inc			=	0.0f;
#ifdef	MEASURE_SPEED_REFIN
	uint32_t	u32_temp;
	float		f_temp;
	uint32_t	u32_temp2;
	float		f_temp2;
#endif
	
	/*��������pid���*/
	spd_in					=	m_motor_rt_para.m_inc_encoder.f_motor_cal_speed;	//��ȡ�����ٶ�ֵ
	pid_inc					=	Increment_PID_Cal((PID_struct*)pid,spd_in);			//*2.0f

#ifdef	MEASURE_SPEED_REFIN
	f_temp					=	m_motor_rt_para.m_inc_encoder.f_motor_cal_speed;	//����Ŀ���ѹ
	u32_temp				=	(uint32_t)(f_temp * 1000.0f + 1500.0f);				//5rps��Ӧ3.3V
	if(u32_temp>4095) u32_temp 		= 	4095;
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);
	
	f_temp2					=	m_motor_ctrl.m_motion_ctrl.f_set_speed;
	u32_temp2				=	(uint32_t)(f_temp2 * 1000.0f + 1500.0f);
	if(u32_temp2>4095) u32_temp2 		= 	4095;
	DAC_SetChannel2Data(DAC_Align_12b_R,(uint16_t)u32_temp2);
#endif
	
	/*��pid���ֵ�ۼӵ���������ֵ*/
	m_motor_ctrl.m_motion_ctrl.f_set_current		=	pid_inc;		
	
	m_motor_ctrl.m_motion_ctrl.u8_current_set_data_refreshed	=	1;									//�����������ݸ���
}

	/*---------------------------------------------------------------------------
	��������			��Position_PID_Cal(volatile PID_Struct * pid)
	��������			��λ�û�pid�ṹ��
	��������			��λ�û���PID����
	----------------------------------------------------------------------------*/
void	Position_PID_Cal(volatile PID_struct * pid)
{
	float		pos_in				=	0.0f;
	float		pid_inc				=	0.0f;

#ifdef	MEASURE_POSITION_REFIN
	uint32_t	u32_temp;
	float		f_temp;
	uint32_t	u32_pos_refin;
	float		f_pos_refin			=	0.0f;
#endif
/*��������pid���*/
	pos_in							=	m_motor_rt_para.m_abs_encoder.f_abs_pos;						//��ȡ����λ��ֵ
	pid_inc							=	Increment_PID_Cal((PID_struct*)pid,pos_in);

#ifdef	MEASURE_POSITION_REFIN
	f_temp							=	(pid->Out_Actual +10.0f)*150.0f;								//�鿴���������
//	f_temp							=	(pos_in - m_motor_rt_para.m_abs_encoder.f_abs_pos_init + 5.0f)*400.0f;											//ʵ��λ��ֵ
	u32_temp						=	(uint32_t)(f_temp);										
	if(u32_temp>4095) u32_temp 		=	4095;
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);
	
	f_pos_refin						=	(m_motor_ctrl.m_motion_ctrl.f_set_position + 5.0 )*400.0f;		//����Ŀ���ֵ
	u32_pos_refin					=	(uint32_t)(f_pos_refin);
	if(u32_pos_refin>4095) u32_pos_refin 		=	4095;
	DAC_SetChannel2Data(DAC_Align_12b_R,(uint16_t)u32_pos_refin);
#endif
	
/*��pid����ֵ��ֵ������ֵ��*/
	m_motor_ctrl.m_motion_ctrl.f_set_speed		=	pid_inc;
	
/*�����ٶȵ��ڵ������ٶ�*/
	
	m_motor_ctrl.m_motion_ctrl.u8_speed_set_data_refreshed	=	1;
}

	/*---------------------------------------------------------------------------
	��������			��(void)
	��������			��null
	��������			��CAN���͵����ٶ�λ��ֵ
	----------------------------------------------------------------------------*/
void	CAN_SEND_CVP(void)
{
	U16 u16_temp;
	uint8_t data[8];
	
	u16_temp.u16_data		=	m_motor_rt_para.m_current_sensor.i16_uvw_current;
	
	data[0] 				=	u16_temp.segment[0];
	data[1] 				=	u16_temp.segment[1];
	
	u16_temp.u16_data		=	(int16_t)(m_motor_rt_para.m_inc_encoder.f_motor_cal_speed*100.0f);
	data[2] 				=	u16_temp.segment[0];
	data[3] 				=	u16_temp.segment[1];
	
	u16_temp.u16_data		=	m_motor_rt_para.m_abs_encoder.u16_abs_cnt;
	data[4] 				=	u16_temp.segment[0];
	data[5] 				=	u16_temp.segment[1];
	
	u16_temp.u16_data		=	(uint16_t)((m_motor_ctrl.m_motion_ctrl.f_set_position + m_motor_rt_para.m_abs_encoder.f_abs_pos_init) * 100.0f);
	data[6] 				=	u16_temp.segment[0];
	data[7] 				=	u16_temp.segment[1];	
	
	//CAN���ͱ��Ĳ���
	while(!CAN1_TX_Data((CanTxMsg*)&(m_can.TxMessage),
						CMD_GET_SPD_POS_CUR,
						(uint8_t*)&data,
						8));
	
}
	
	
	
	/*---------------------------------------------------------------------------
	��������			��(void)
	��������			��null
	��������			��PID����
	----------------------------------------------------------------------------*/





/*********************************************END OF FILE**************************************************/

