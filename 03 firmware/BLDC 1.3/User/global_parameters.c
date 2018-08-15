/**
  ******************************************************************************
  * @file    Project/ucosii stm32f4 mdk475/global_parameters.h 
  * @author  Kuangjing
  * @version ucosii
  * @date    20180619
  * @brief   ����ȫ�ֱ�����������������Ʋ�����.
  ******************************************************************************
  ******************************************************************************
  */

#include	"global_parameters.h"
#include	"stm32f4xx.h"
#include	"motor_control.h"

sys_state				m_sys_state;

const_motor_para		m_motor;

motor_runtime_para		m_motor_rt_para;

motor_control_para		m_motor_ctrl;

current_pid_para		m_current_pid;

speed_pid_para			m_speed_pid;

position_pid_para		m_position_pid;

error_log				m_error;


	/*---------------------------------------------------------------------------
	��������			��ParametersInit(void)
	��������			��null
	��������			����ʼ��������
	----------------------------------------------------------------------------*/
uint8_t	ParametersInit(void)
{
	m_sys_state.u8_cur_state				=	Idle_state;								//��ʼʱ���״̬Ϊidle
	m_sys_state.u8_pre_state				=	Idle_state;
	
	m_motor_ctrl.u8_dir						=	1;										//���õ��ת��
	
	//���������ʼ��
	m_motor.m_encoder.u32_line_per_loop		=	LINES_PER_LOOP;
	m_motor.m_encoder.u8_multiplication		=	MULTIPLICATION;
	m_motor.m_encoder.u32_pulse_per_loop	=	LINES_PER_LOOP * MULTIPLICATION;
	
	m_motor.u32_maxspeed					=	(uint32_t)(V_bus/Kb_cold*1000);
	m_motor.f_no_load_speed					=	N_Rated;
	m_motor.f_no_load_current				=	Ic;
	m_motor.f_max_current					=	Ip;
	m_motor.f_stall_torque					=	Tp;
	m_motor.f_speed_constant				=	1000.0f/Kb_hot;
	m_motor.f_torque_constant				=	1000.0f*Kt_hot;
	m_motor.f_current_constant				=	1.0f/m_motor.f_torque_constant;
	m_motor.f_terminal_inductance			=	Lm*1000;
	m_motor.f_mechanical_time_constant		=	0.0f;
	m_motor.u8_number_of_pole_pairs			=	P;
	
	//������pid������ʼ��
	m_current_pid.curr_pid.Ref_In			=	0.1f;
	m_current_pid.curr_pid.Feed_Back		=	0.0f;
	m_current_pid.curr_pid.Err_T_0			=	0.0f;
	m_current_pid.curr_pid.Err_T_1			=	0.0f;
	m_current_pid.curr_pid.Err_T_2			=	0.0f;
	m_current_pid.curr_pid.Err_Dif			=	0.0f;
	
	m_current_pid.curr_pid.P_Out			=	0.0f;
	m_current_pid.curr_pid.I_Out			=	0.0f;
	m_current_pid.curr_pid.D_Out			=	0.0f;
	m_current_pid.curr_pid.Out_Pre			=	0.0f;
	m_current_pid.curr_pid.Out_Actual		=	0.0f;
	
	m_current_pid.curr_pid.Kp				=	0.1f;
	m_current_pid.curr_pid.Ki				=	0.001f;
	m_current_pid.curr_pid.Kd				=	0.0f;
	
	//�ٶȻ�pid������ʼ��
	m_speed_pid.spd_pid.Ref_In				=	0.1f;
	m_speed_pid.spd_pid.Feed_Back			=	0.0f;
	m_speed_pid.spd_pid.Err_T_0				=	0.0f;
	m_speed_pid.spd_pid.Err_T_1				=	0.0f;
	m_speed_pid.spd_pid.Err_T_2				=	0.0f;
	m_speed_pid.spd_pid.Err_Dif				=	0.0f;
	
	m_speed_pid.spd_pid.P_Out				=	0.0f;
	m_speed_pid.spd_pid.I_Out				=	0.0f;
	m_speed_pid.spd_pid.D_Out				=	0.0f;
	m_speed_pid.spd_pid.Out_Pre				=	0.0f;
	m_speed_pid.spd_pid.Out_Actual			=	0.0f;
	
	m_speed_pid.spd_pid.Kp					=	2.8f;
	m_speed_pid.spd_pid.Ki					=	0.0001f;
	m_speed_pid.spd_pid.Kd					=	0.0f;
	
	
	return	1;
}


	/*---------------------------------------------------------------------------
	��������			��CurrentFilterDataInit(void)
	��������			��null
	��������			���������˲����������
							�ȹرյ���������
							���˲����������
							�򿪵�����
							�����˲���������ʼ��䣬����ƫ�õļ���
	----------------------------------------------------------------------------*/
void	CurrentFilterDataInit(void)
{
	uint8_t	i,j;
	//ֹͣ����������
	CurrentLoopRefresh_TIM_Halt();
	//�˲����������
	for(i=0;i<3;i++)
	{
		m_motor_rt_para.u16_uvw_sum[i]			=	0;
		m_motor_rt_para.filter_index_uvw[i]		=	0;
		m_motor_rt_para.uvw_buffer_used[i]		=	0;
		for(j=0;j<CURRENT_BUFFER_LENGTH;j++)
		{
			m_motor_rt_para.history_data[i][j]	=	0;
		}
		m_motor_rt_para.u16_uvw_current[i]		=	0;
		m_motor_rt_para.u16_uvw_curr_bias[i]	=	0;
		m_motor_rt_para.f_adc_UVW_I[i]			=	0.0f;		
	}

	//����ƫ�ã��ڲ��д򿪵���������
	Read_Current_Bias();
}




	/*---------------------------------------------------------------------------
	��������			��EncoderDataInit(void)
	��������			��null
	��������			��������������������
	----------------------------------------------------------------------------*/
void	EncoderDataInit(void)
{
	m_motor_rt_para.i32_encoder_last_read	=	0;
	m_motor_rt_para.i32_encoder_curr_read	=	0;
	m_motor_rt_para.i64_pulse_cnt			=	0;
	m_motor_rt_para.u32_pulse_width			=	0;
	m_motor_rt_para.f_motor_cal_speed		=	0.0f;
	m_motor_rt_para.f_shaft_cal_speed		=	0.0f;
	
}


	/*---------------------------------------------------------------------------
	��������			��ParametersSave(void)
	��������			��null
	��������			����������������flash��
	----------------------------------------------------------------------------*/
uint8_t	ParametersSave(void)
{

	return	1;
}


	/*---------------------------------------------------------------------------
	��������			��ParametersCheck(void)
	��������			��null
	��������			������������Ƿ�Ϸ�
	----------------------------------------------------------------------------*/
uint8_t	ParametersCheck(void)
{

	return	1;
}





/**********************end of file************************/

