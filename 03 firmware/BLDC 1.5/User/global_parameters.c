/**
  ******************************************************************************
  * @file    Project/ucosii stm32f4 mdk475/global_parameters.h 
  * @author  Kuangjing
  * @version ucosii
  * @date    20180619
  * @brief   定义全局变量，电机参数，控制参数等.
  ******************************************************************************
  ******************************************************************************
  */

#include	"global_parameters.h"
#include	"stm32f4xx.h"
#include	"motor_control.h"
#include	"spd_pos_filter.h"
#include	"node_can_config.h"


servo_motor_attribute_para		m_motor_attribute_para;

motor_runtime_para				m_motor_rt_para;

motor_control_state				m_motor_ctrl;

motor_pid_control				m_pid;

can_bus							m_can;
	
error_log						m_error;



	/*---------------------------------------------------------------------------
	函数名称			：IndirectParaInit(void)
	参数含义			：null
	函数功能			：由固定参数计算的控制数据
	----------------------------------------------------------------------------*/
void	IndirectParaInit(void)
{
	m_motor_attribute_para.m_motor_ind_att.f_Udc			=	24.0f;
	m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc	=	41.56921938165f * PWM_CYCLE_F;
	m_motor_attribute_para.m_motor_ind_att.f_rad_per_cnts	=	6.283185307f/(float)(m_motor_attribute_para.m_encoder_att.u32_pulse_per_loop / m_motor_attribute_para.m_motor_att.u8_number_of_pole_pairs);
}

	/*---------------------------------------------------------------------------
	函数名称			：CurrentPID6StepsDataInit(void)
	参数含义			：null
	函数功能			：六步换向时的电流环pid数据初始化
	----------------------------------------------------------------------------*/
void	CurrentPID6StepsDataInit(void)
{
	/*电流环pid参数初始化*/
	m_pid.curr.Ref_In			=	0.0f;
	m_pid.curr.Feed_Back		=	0.0f;
	m_pid.curr.Err_T_0			=	0.0f;
	m_pid.curr.Err_T_1			=	0.0f;
	m_pid.curr.Err_T_2			=	0.0f;
	m_pid.curr.Err_Dif			=	0.0f;
	
	m_pid.curr.P_Out			=	0.0f;
	m_pid.curr.I_Out			=	0.0f;
	m_pid.curr.D_Out			=	0.0f;
	m_pid.curr.Out_Pre			=	0.0f;
	m_pid.curr.Out_Actual		=	0.0f;
	
	m_pid.curr.Kp				=	2.0f;
	m_pid.curr.Ki				=	0.02f;//0.01
	m_pid.curr.Kd				=	0.0f;
//pid: 20.0 1.0 0.0
//pid: 40.0 0.8 0.0
//
//
}

	/*---------------------------------------------------------------------------
	函数名称			：CurrentPIDSVPWMDataInit(void)
	参数含义			：null
	函数功能			：电流环pid数据初始化
	----------------------------------------------------------------------------*/
void	CurrentPIDSVPWMDataInit(void)
{
	m_pid.id.Ref_In				=	0.0f;
	m_pid.id.Feed_Back			=	0.0f;
	m_pid.id.Err_T_0			=	0.0f;
	m_pid.id.Err_T_1			=	0.0f;
	m_pid.id.Err_T_2			=	0.0f;
	m_pid.id.Err_Dif			=	0.0f;
	
	m_pid.id.P_Out				=	0.0f;
	m_pid.id.I_Out				=	0.0f;
	m_pid.id.D_Out				=	0.0f;
	m_pid.id.Out_Pre			=	0.0f;
	m_pid.id.Out_Actual			=	0.0f;
	
	m_pid.id.Kp					=	8.0f;
	m_pid.id.Ki					=	0.4f;
	m_pid.id.Kd					=	0.0f;
	
	m_pid.iq.Ref_In				=	0.0f;
	m_pid.iq.Feed_Back			=	0.0f;
	m_pid.iq.Err_T_0			=	0.0f;
	m_pid.iq.Err_T_1			=	0.0f;
	m_pid.iq.Err_T_2			=	0.0f;
	m_pid.iq.Err_Dif			=	0.0f;
	
	m_pid.iq.P_Out				=	0.0f;
	m_pid.iq.I_Out				=	0.0f;
	m_pid.iq.D_Out				=	0.0f;
	m_pid.iq.Out_Pre			=	0.0f;
	m_pid.iq.Out_Actual			=	0.0f;
	
	m_pid.iq.Kp					=	8.0f;
	m_pid.iq.Ki					=	0.4f;
	m_pid.iq.Kd					=	0.0f;	
}


	/*---------------------------------------------------------------------------
	函数名称			：SpeedPIDDataInit(void)
	参数含义			：null
	函数功能			：速度环pid数据初始化
	----------------------------------------------------------------------------*/
void	SpeedPIDDataInit(void)
{
	/*速度环pid参数初始化*/
	m_pid.spd.Ref_In			=	0.0f;
	m_pid.spd.Feed_Back			=	0.0f;
	m_pid.spd.Err_T_0			=	0.0f;
	m_pid.spd.Err_T_1			=	0.0f;
	m_pid.spd.Err_T_2			=	0.0f;
	m_pid.spd.Err_Dif			=	0.0f;
	
	m_pid.spd.P_Out				=	0.0f;
	m_pid.spd.I_Out				=	0.0f;
	m_pid.spd.D_Out				=	0.0f;
	m_pid.spd.Out_Pre			=	0.0f;
	m_pid.spd.Out_Actual		=	0.0f;
	
	m_pid.spd.Kp				=	8.0f;
	m_pid.spd.Ki				=	0.4f;
	m_pid.spd.Kd				=	0.0f;
//pid: 8.0 0.7 0.5 毛刺较大
//pid: 3.2 0.41 0.0
//pid: 2.0 0.05 0.0
//
}


	/*---------------------------------------------------------------------------
	函数名称			：PosPIDDataInit(void)
	参数含义			：null
	函数功能			：位置环pid数据初始化
	----------------------------------------------------------------------------*/
void	PosPIDDataInit(void)
{
	/*位置环pid参数初始化*/
	m_pid.pos.Ref_In			=	0.0f;
	m_pid.pos.Feed_Back			=	0.0f;
	m_pid.pos.Err_T_0			=	0.0f;
	m_pid.pos.Err_T_1			=	0.0f;
	m_pid.pos.Err_T_2			=	0.0f;
	m_pid.pos.Err_Dif			=	0.0f;
	
	m_pid.pos.P_Out				=	0.0f;
	m_pid.pos.I_Out				=	0.0f;
	m_pid.pos.D_Out				=	0.0f;
	m_pid.pos.Out_Pre			=	0.0f;
	m_pid.pos.Out_Actual		=	0.0f;
	
	m_pid.pos.Kp				=	0.4f;
	m_pid.pos.Ki				=	0.04f;
	m_pid.pos.Kd				=	0.1f;
//pid 0.4 0.05 0.1
//
//
}

	/*---------------------------------------------------------------------------
	函数名称			：EncoderDataInit(void)
	参数含义			：null
	函数功能			：将电机编码器数据清除
	----------------------------------------------------------------------------*/
void	EncoderDataInit(void)
{
	uint8_t	i;
	m_motor_rt_para.m_inc_encoder.u8_M_or_T						=	T_METHORD;
	m_motor_rt_para.m_inc_encoder.u8_speed_filter_used			=	0;
	m_motor_rt_para.m_inc_encoder.i32_spd_his_sum				=	0;
	m_motor_rt_para.m_inc_encoder.u16_encoder_last_read			=	0;
	m_motor_rt_para.m_inc_encoder.u16_encoder_curr_read			=	0;
	m_motor_rt_para.m_inc_encoder.i32_pulse_cnt					=	0;
	m_motor_rt_para.m_inc_encoder.u32_pulse_width				=	0;
	m_motor_rt_para.m_inc_encoder.f_motor_cal_speed				=	0.0f;
	m_motor_rt_para.m_inc_encoder.f_shaft_cal_speed				=	0.0f;
	for(i=0;i<SPEED_BUFFER_LENGTH;i++)
		m_motor_rt_para.m_inc_encoder.i32_spd_hisdata[i]		=	0;
	
}



	/*---------------------------------------------------------------------------
	函数名称			：MotorCtrlDataInit(void)
	参数含义			：null
	函数功能			：驱动器内三环的标志位与设置值数据初始化
	----------------------------------------------------------------------------*/
void	MotorCtrlDataInit(void)
{
	m_motor_ctrl.m_motion_ctrl.u8_dir								=	0;
	m_motor_ctrl.m_motion_ctrl.u8_current_read_data_refreshed		=	0;
	m_motor_ctrl.m_motion_ctrl.u8_current_set_data_refreshed		=	0;
	m_motor_ctrl.m_motion_ctrl.u8_speed_read_data_refreshed			=	0;
	m_motor_ctrl.m_motion_ctrl.u8_speed_set_data_refreshed			=	0;
	m_motor_ctrl.m_motion_ctrl.u8_position_read_data_refreshed		=	0;
	m_motor_ctrl.m_motion_ctrl.u8_position_set_data_refreshed		=	1;
	
	m_motor_ctrl.m_motion_ctrl.f_set_current						=	0.0f;
	m_motor_ctrl.m_motion_ctrl.f_set_speed							=	0.0f;	//3.0f
	m_motor_ctrl.m_motion_ctrl.f_set_position						=	0.0f;
	
	m_motor_ctrl.m_motion_ctrl.i32_set_current						=	0;
	m_motor_ctrl.m_motion_ctrl.i32_set_speed						=	0;
	m_motor_ctrl.m_motion_ctrl.i32_set_position						=	0;
	
	m_motor_ctrl.m_motion_ctrl.u8_is_currloop_used					=	0;
	m_motor_ctrl.m_motion_ctrl.u8_is_speedloop_used					=	0;
	m_motor_ctrl.m_motion_ctrl.u8_is_posloop_used					=	0;
}



	/*---------------------------------------------------------------------------
	函数名称			：ParametersInit(void)
	参数含义			：null
	函数功能			：初始化各参数
	----------------------------------------------------------------------------*/
uint8_t	ParametersInit(void)
{
	
	m_motor_ctrl.m_sys_state.u8_cur_state									=	Idle_state;								//初始时电机状态为idle
	m_motor_ctrl.m_sys_state.u8_pre_state									=	Idle_state;
	
	/*电机参数初始化*/
	m_motor_attribute_para.m_encoder_att.u32_line_per_loop				=	LINES_PER_LOOP;
	m_motor_attribute_para.m_encoder_att.u8_multiplication				=	MULTIPLICATION;
	m_motor_attribute_para.m_encoder_att.u32_pulse_per_loop				=	LINES_PER_LOOP * MULTIPLICATION;
	m_motor_attribute_para.m_motor_att.u32_maxspeed						=	(uint32_t)(V_bus/Kb_cold*1000);
	m_motor_attribute_para.m_motor_att.f_no_load_speed					=	N_Rated;
	m_motor_attribute_para.m_motor_att.f_no_load_current				=	Ic;
	m_motor_attribute_para.m_motor_att.f_max_current					=	Ip;
	m_motor_attribute_para.m_motor_att.f_stall_torque					=	Tp;
	m_motor_attribute_para.m_motor_att.f_speed_constant					=	1000.0f/Kb_hot;
	m_motor_attribute_para.m_motor_att.f_torque_constant				=	1000.0f*Kt_hot;
	m_motor_attribute_para.m_motor_att.f_current_constant				=	1.0f/m_motor_attribute_para.m_motor_att.f_torque_constant;
	m_motor_attribute_para.m_motor_att.f_terminal_inductance			=	Lm*1000;
	m_motor_attribute_para.m_motor_att.f_mechanical_time_constant		=	0.0f;
	m_motor_attribute_para.m_motor_att.u8_number_of_pole_pairs			=	P/2;
	

	CurrentPID6StepsDataInit();
	CurrentPIDSVPWMDataInit();
	SpeedPIDDataInit();
	PosPIDDataInit();

	CurrentFilterDataInit();									//电流滤波器参数初始化
	EncoderDataInit();											//编码器数据初始化
	MotorCtrlDataInit();										//电机控制数据初始化
	KF_Filter_Init(&m_KF);										//速度位置预估卡尔曼滤波器初始化
	SystemStateDataInit();										//系统状态相关数据初始化设置
	return	1;
}


	/*---------------------------------------------------------------------------
	函数名称			：CurrentFilterDataInit(void)
	参数含义			：null
	函数功能			：将电流滤波器数据清除
							先关闭电流环更新
							将滤波器数据清除
							打开电流环
							进行滤波器参数初始填充，包括偏置的计算
	----------------------------------------------------------------------------*/
void	CurrentFilterDataInit(void)
{
	uint8_t	i,j;
	//停止电流环更新
	CurrentLoopRefresh_TIM_Halt();
	//滤波器数据清除

	m_motor_rt_para.m_current_filter.i16_uvw_sum				=	0;
	m_motor_rt_para.m_current_filter.filter_index_uvw			=	0;
	m_motor_rt_para.m_current_filter.uvw_buffer_used			=	0;
	for(i=0;i<3;i++)
	{
		for(j=0;j<CURRENT_BUFFER_LENGTH;j++)
		{
			m_motor_rt_para.m_current_filter.history_data[i][j]		=	0;
		}
	}
	m_motor_rt_para.m_current_sensor.i16_uvw_current			=	0;
	m_motor_rt_para.m_current_sensor.i16_uvw_curr_bias			=	0;
	m_motor_rt_para.m_current_sensor.f_adc_UVW_I				=	0.0f;		

}





	/*---------------------------------------------------------------------------
	函数名称			：SystemStateDataInit(void)
	参数含义			：null
	函数功能			：系统状态数据初始化
	----------------------------------------------------------------------------*/
void	SystemStateDataInit(void)
{
	m_motor_ctrl.m_sys_state.u8_abs_encoder_used					=	1;
	m_motor_ctrl.m_sys_state.u8_cur_state							=	Idle_state;
	m_motor_ctrl.m_sys_state.u8_pre_state							=	Idle_state;
	m_motor_ctrl.m_sys_state.u32_node_id							=	NODE_ID;
	m_motor_ctrl.m_sys_state.u8_use_svpwm							=	USE_FOC;
}
	/*---------------------------------------------------------------------------
	函数名称			：ParametersSave(void)
	参数含义			：null
	函数功能			：将电机参数存放至flash中
	----------------------------------------------------------------------------*/
uint8_t	ParametersSave(void)
{
	
	return	1;
}


	/*---------------------------------------------------------------------------
	函数名称			：ParametersCheck(void)
	参数含义			：null
	函数功能			：检查电机参数是否合法
	----------------------------------------------------------------------------*/
uint8_t	ParametersCheck(void)
{

	return	1;
}





/**********************end of file************************/

