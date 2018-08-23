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
	
	/*���������ʼ��*/
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
	
	/*������pid������ʼ��*/
	m_current_pid.curr_pid.Ref_In			=	0.0f;
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
	
	m_current_pid.curr_pid.Kp				=	0.6f;
	m_current_pid.curr_pid.Ki				=	0.02f;
	m_current_pid.curr_pid.Kd				=	0.0f;
//pid: 20.0 1.0 0.0
//pid: 40.0 0.8 0.0
//
//
	/*�ٶȻ�pid������ʼ��*/
	m_speed_pid.spd_pid.Ref_In				=	0.0f;
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
	
	m_speed_pid.spd_pid.Kp					=	4.0f;
	m_speed_pid.spd_pid.Ki					=	0.1f;
	m_speed_pid.spd_pid.Kd					=	0.0f;
//pid: 8.0 0.7 0.5 ë�̽ϴ�
//pid: 3.2 0.41 0.0
//
//
	/*λ�û�pid������ʼ��*/
	m_position_pid.pos_pid.Ref_In			=	0.0f;
	m_position_pid.pos_pid.Feed_Back		=	0.0f;
	m_position_pid.pos_pid.Err_T_0			=	0.0f;
	m_position_pid.pos_pid.Err_T_1			=	0.0f;
	m_position_pid.pos_pid.Err_T_2			=	0.0f;
	m_position_pid.pos_pid.Err_Dif			=	0.0f;
	
	m_position_pid.pos_pid.P_Out			=	0.0f;
	m_position_pid.pos_pid.I_Out			=	0.0f;
	m_position_pid.pos_pid.D_Out			=	0.0f;
	m_position_pid.pos_pid.Out_Pre			=	0.0f;
	m_position_pid.pos_pid.Out_Actual		=	0.0f;
	
	m_position_pid.pos_pid.Kp				=	0.004f;
	m_position_pid.pos_pid.Ki				=	0.00001f;
	m_position_pid.pos_pid.Kd				=	0.001f;
//pid 0.004 0.00005 0.002
//
//
	CurrentFilterDataInit();
	EncoderDataInit();
	MotorCtrlDataInit();
	
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
	uint8_t	j;
	//ֹͣ����������
	CurrentLoopRefresh_TIM_Halt();
	//�˲����������

	m_motor_rt_para.u16_uvw_sum				=	0;
	m_motor_rt_para.filter_index_uvw		=	0;
	m_motor_rt_para.uvw_buffer_used			=	0;
	for(j=0;j<CURRENT_BUFFER_LENGTH;j++)
	{
		m_motor_rt_para.history_data[j]		=	0;
	}
	m_motor_rt_para.u16_uvw_current			=	0;
	m_motor_rt_para.u16_uvw_curr_bias		=	0;
	m_motor_rt_para.f_adc_UVW_I				=	0.0f;		

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
	uint8_t	i;
	m_motor_rt_para.m_encoder.u8_M_or_T				=	T_METHORD;
	m_motor_rt_para.m_encoder.u8_speed_filter_used	=	0;
	m_motor_rt_para.m_encoder.i32_spd_his_sum		=	0;
	m_motor_rt_para.m_encoder.u16_encoder_last_read	=	0;
	m_motor_rt_para.m_encoder.u16_encoder_curr_read	=	0;
	m_motor_rt_para.m_encoder.i32_pulse_cnt			=	0;
	m_motor_rt_para.m_encoder.u32_pulse_width		=	0;
	m_motor_rt_para.m_encoder.f_motor_cal_speed		=	0.0f;
	m_motor_rt_para.m_encoder.f_shaft_cal_speed		=	0.0f;
	for(i=0;i<SPEED_BUFFER_LENGTH;i++)
		m_motor_rt_para.m_encoder.i32_spd_hisdata[i]=	0;
	
}


	/*---------------------------------------------------------------------------
	��������			��MotorCtrlDataInit(void)
	��������			��null
	��������			���������������ı�־λ������ֵ���ݳ�ʼ��
	----------------------------------------------------------------------------*/
void	MotorCtrlDataInit(void)
{
	m_motor_ctrl.u8_dir								=	0;
	m_motor_ctrl.u8_current_read_data_refreshed		=	0;
	m_motor_ctrl.u8_current_set_data_refreshed		=	0;
	m_motor_ctrl.u8_speed_read_data_refreshed		=	0;
	m_motor_ctrl.u8_speed_set_data_refreshed		=	0;
	m_motor_ctrl.u8_position_read_data_refreshed	=	0;
	m_motor_ctrl.u8_position_set_data_refreshed		=	1;
	
	m_motor_ctrl.f_set_current						=	0.0f;
	m_motor_ctrl.f_set_speed						=	0.0f;	//3.0f
	m_motor_ctrl.f_set_position						=	0.0f;
	
	m_motor_ctrl.i32_set_current					=	0;
	m_motor_ctrl.i32_set_speed						=	0;
	m_motor_ctrl.i32_set_position					=	0;
	
	m_motor_ctrl.u8_is_currloop_used				=	0;
	m_motor_ctrl.u8_is_speedloop_used				=	0;
	m_motor_ctrl.u8_is_posloop_used					=	0;
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

