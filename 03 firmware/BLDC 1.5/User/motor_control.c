/**
  ******************************************************************************
  * @file    Project/user/motor_control.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170728
  * @brief   电机控制相关函数，如PWM的产生，换向，PID计算，编码器读取等
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
	函数名称			：Increment_PID_Cal(PID_Struct* pid)
	参数含义			：null
	函数功能			：增量PID计算过程，输出值在-10到10之间
							本地函数
	----------------------------------------------------------------------------*/
float	Increment_PID_Cal(PID_struct* pid,float new_feedback)
{
	pid->Feed_Back			=	new_feedback;													//更新反馈值
	pid->Err_T_2			=	pid->Err_T_1;													//更新T-2时刻误差
	pid->Err_T_1			=	pid->Err_T_0;													//更新T-1时刻误差
	pid->Err_T_0			=	pid->Ref_In - pid->Feed_Back;									//计算新的误差
	
	pid->P_Out				=	pid->Kp * (pid->Err_T_0 - pid->Err_T_1);						//比例部分
	pid->I_Out				=	pid->Ki * pid->Err_T_0;											//积分部分
	pid->D_Out				=	pid->Kd * (pid->Err_T_0 - 2.0f * pid->Err_T_1 + pid->Err_T_2);	//微分部分
	
	pid->pid_increment		=	pid->P_Out + pid->I_Out + pid->D_Out;							//计算PID增量
	
	pid->Out_Actual			=	pid->Out_Pre + pid->pid_increment;
	
	if(pid->Out_Actual > 10.0f)
		pid->Out_Actual		=	10.0f;
	if(pid->Out_Actual < -10.0f)
		pid->Out_Actual		=	-10.0f;
	
	pid->Out_Pre			=	pid->Out_Actual;
	
	return	pid->Out_Actual;
}

/*反馈值使用int型的计算pid*/
float	Increment_PID_Cal_int(PID_struct* pid, int32_t new_feedback)
{
	pid->Feed_Back			=	(float)new_feedback;													//更新反馈值
	pid->Err_T_2			=	pid->Err_T_1;													//更新T-2时刻误差
	pid->Err_T_1			=	pid->Err_T_0;													//更新T-1时刻误差
	pid->Err_T_0			=	pid->Ref_In - pid->Feed_Back;									//计算新的误差
	
	pid->P_Out				=	pid->Kp * (pid->Err_T_0 - pid->Err_T_1);						//比例部分
	pid->I_Out				=	pid->Ki * pid->Err_T_0;											//积分部分
	pid->D_Out				=	pid->Kd * (pid->Err_T_0 - 2.0f * pid->Err_T_1 + pid->Err_T_2);	//微分部分
	
	pid->pid_increment		=	pid->P_Out + pid->I_Out + pid->D_Out;							//计算PID增量
	
	pid->Out_Actual			=	pid->Out_Pre + pid->pid_increment;
	
	if(pid->Out_Actual > 10.0f)
		pid->Out_Actual		=	10.0f;
	if(pid->Out_Actual < -10.0f)
		pid->Out_Actual		=	-10.0f;
	
	pid->Out_Pre			=	pid->Out_Actual;
	
	return	pid->Out_Actual;
}


	/*---------------------------------------------------------------------------
	函数名称			：Absolute_PID_Cal(PID_struct* pid)
	参数含义			：null
	函数功能			：绝对值式PID计算过程，输出值在-10到10之间
							本地函数
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
	函数名称			：Hall_Convert(void)
	参数含义			：null
	函数功能			：初始启动时Hall换向函数
	----------------------------------------------------------------------------*/
void	Hall_Start_Convert(void)
{
	m_motor_rt_para.m_reverse.u8_hall_state		=	Hall_State_Read();							//记录hall状态
	
	startup_switch_table[m_motor_ctrl.m_motion_ctrl.u8_dir][m_motor_rt_para.m_reverse.u8_hall_state]();		//根据方向以及hall相位进行换向，采用函数指针方式
}


	/*---------------------------------------------------------------------------
	函数名称			：Hall_Runtime_Convert(void)
	参数含义			：null
	函数功能			：运行时Hall换向函数
	----------------------------------------------------------------------------*/
void	Hall_Runtime_Convert(void)
{
	m_motor_rt_para.m_reverse.u8_hall_state		=	Hall_State_Read();							//记录hall状态
	
	runtime_switch_table[m_motor_ctrl.m_motion_ctrl.u8_dir][m_motor_rt_para.m_reverse.u8_hall_state]();		//根据方向以及hall相位进行换向，采用函数指针方式
}


	/*---------------------------------------------------------------------------
	函数名称			：PWM_TIM_Start(void)
	参数含义			：null
	函数功能			：开启定时器产生PWM
	----------------------------------------------------------------------------*/
void	PWM_TIM_Start(void)
{
	TIM_Cmd(TIM1,ENABLE);															//使能定时器1
}




	/*---------------------------------------------------------------------------
	函数名称			：PWM_TIM_Halt(void)
	参数含义			：null
	函数功能			：暂停定时器产生PWM
	----------------------------------------------------------------------------*/
void	PWM_TIM_Halt(void)
{
	TIM_Cmd(TIM1,DISABLE);																		//禁用定时器1
}



	/*---------------------------------------------------------------------------
	函数名称			：CoilCurrentRefresh_TIM_Start(void)
	参数含义			：null
	函数功能			：开启电流环的更新定时器中断
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
	函数名称			：CoilCurrentRefresh_TIM_Halt(void)
	参数含义			：null
	函数功能			：关闭电流环的更新定时器中断
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
	函数名称			：SpeedPosLoopRefresh_TIM_Start(void)
	参数含义			：null
	函数功能			：	开启速度环的更新定时器中断
							开启速度环必须开启电流环
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
	函数名称			：SpeedPosLoopRefresh_TIM_Halt(void)
	参数含义			：null
	函数功能			：开启速度环的更新定时器中断
	----------------------------------------------------------------------------*/
void	SpeedLoopRefresh_TIM_Halt(void)
{
	m_motor_ctrl.m_motion_ctrl.u8_is_speedloop_used	=	0;
}



	/*---------------------------------------------------------------------------
	函数名称			：PositionLoopRefresh_TIM_Start(void)
	参数含义			：null
	函数功能			：开启位置环的更新定时器中断
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
	函数名称			：PositionLoopRefresh_TIM_Start(void)
	参数含义			：null
	函数功能			：关闭位置环的更新
	----------------------------------------------------------------------------*/
void	PositionLoopRefresh_TIM_Halt(void)
{
	m_motor_ctrl.m_motion_ctrl.u8_is_posloop_used		=	0;
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



	/*---------------------------------------------------------------------------
	函数名称			：Read_Current_Bias(void)
	参数含义			：null
	函数功能			：获取电流测量值偏置
	----------------------------------------------------------------------------*/
void	Read_Current_Bias(void)
{
	uint8_t	i;
	
	if(m_motor_ctrl.m_sys_state.u8_cur_state != Prepare_state)						//更改状态机状态
		m_motor_ctrl.m_sys_state.u8_cur_state = Prepare_state;
	
	IR2130S_force_reset();															//占空比为0
		
	CurrentLoopRefresh_TIM_Halt();													//暂停电流环更新
	
	Current_Filter_Init();															//初始化电流滤波器

	CurrentLoopRefresh_TIM_Start();
}


	/*---------------------------------------------------------------------------
	函数名称			：Curr_PID_Cal(volatile PID_Struct * pid)
	参数含义			：null
	函数功能			：	电流环PID计算，电流环输入只有正值，仅进行PWM占空比的设置
						增量式PID计算
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
	
	/*判断输入电流值是否在额定电流内*/
	curr_in					=	m_motor_rt_para.m_current_sensor.f_adc_UVW_I;
	if(curr_in > m_motor_attribute_para.m_motor_att.f_max_current)
		curr_in	 			=	m_motor_attribute_para.m_motor_att.f_max_current;
	
	/*计算增量pid输出*/
	pid_inc					=	Increment_PID_Cal((PID_struct*)pid,curr_in);			//暂时使用PI

//	pid_inc					=	Absolute_PID_Cal((PID_struct*)pid,curr_in);
#ifdef	MEASURE_CURRENT_REFIN
	//绿
	f_temp					=	m_pid.curr.Feed_Back;									//跟踪目标电压 绿
	u32_temp				=	(uint32_t)(f_temp * 1241.21f);
	if(u32_temp>4095) u32_temp = 4095;													//将数字量限定幅值
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);							//开启DA转换
	
	//黄
	f_temp2					=	(float) (pid_inc + 10.0f) * 200.0f;//(m_motor_rt_para.m_current_sensor.ADC_DMA_buf[2]);//
	u32_temp2				=	(uint32_t)f_temp2;
	if(u32_temp2>4095) u32_temp2 = 4095;
	DAC_SetChannel2Data(DAC_Align_12b_R,(uint16_t)u32_temp2);
#endif

	/*将pid输出值转化到10-4190之间*/
	m_pid.PW				=	pid_inc * 420.0f;							
	
	/*由设置电流值进行换向*/
	if(m_pid.PW > 1.5f)
	{
		m_motor_ctrl.m_motion_ctrl.u8_dir			=	1;
	}
	else if(m_pid.PW < -1.5f)
	{
		m_motor_ctrl.m_motion_ctrl.u8_dir			=	0;
	}
	
	/*换向*/
	startup_switch_table[m_motor_ctrl.m_motion_ctrl.u8_dir][m_motor_rt_para.m_reverse.u8_hall_state]();
	
	arm_abs_f32(&m_pid.PW,&m_pid.PW,1);
	
	/*对PID输出做出赋值限制*/
	if(m_pid.PW > MAX_DUTY_CYCLE)
		m_pid.PW	=	MAX_DUTY_CYCLE;
	else if(m_pid.PW < MIN_DUTY_CYCLE)
		m_pid.PW	=	MIN_DUTY_CYCLE;
	else
		;
	
	/*更改PWM占空比*/
	ccr						=	(uint16_t)m_pid.PW;
	TIM1->CCR1				=	ccr;
	TIM1->CCR2				=	ccr;
	TIM1->CCR3				=	ccr;
}




	/*---------------------------------------------------------------------------
	函数名称			：Speed_PID_Cal(volatile PID_Struct * pid)
	参数含义			：速度环pid结构体
	函数功能			：速度环的PID计算
	----------------------------------------------------------------------------*/
void	Speed_PID_Cal(volatile PID_struct * pid)
{
	float	spd_in			=	0.0f;												//获取的速度值
	float	pid_inc			=	0.0f;
#ifdef	MEASURE_SPEED_REFIN
	uint32_t	u32_temp;
	float		f_temp;
	uint32_t	u32_temp2;
	float		f_temp2;
#endif
	
	/*计算增量pid输出*/
	spd_in					=	m_motor_rt_para.m_inc_encoder.f_motor_cal_speed;	//获取反馈速度值
	pid_inc					=	Increment_PID_Cal((PID_struct*)pid,spd_in);			//*2.0f

#ifdef	MEASURE_SPEED_REFIN
	f_temp					=	m_motor_rt_para.m_inc_encoder.f_motor_cal_speed;	//跟踪目标电压
	u32_temp				=	(uint32_t)(f_temp * 1000.0f + 1500.0f);				//5rps对应3.3V
	if(u32_temp>4095) u32_temp 		= 	4095;
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);
	
	f_temp2					=	m_motor_ctrl.m_motion_ctrl.f_set_speed;
	u32_temp2				=	(uint32_t)(f_temp2 * 1000.0f + 1500.0f);
	if(u32_temp2>4095) u32_temp2 		= 	4095;
	DAC_SetChannel2Data(DAC_Align_12b_R,(uint16_t)u32_temp2);
#endif
	
	/*将pid输出值累加到电流设置值*/
	m_motor_ctrl.m_motion_ctrl.f_set_current		=	pid_inc;		
	
	m_motor_ctrl.m_motion_ctrl.u8_current_set_data_refreshed	=	1;									//电流设置数据更新
}

	/*---------------------------------------------------------------------------
	函数名称			：Position_PID_Cal(volatile PID_Struct * pid)
	参数含义			：位置环pid结构体
	函数功能			：位置环的PID计算
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
/*计算增量pid输出*/
	pos_in							=	m_motor_rt_para.m_abs_encoder.f_abs_pos;						//获取反馈位置值
	pid_inc							=	Increment_PID_Cal((PID_struct*)pid,pos_in);

#ifdef	MEASURE_POSITION_REFIN
	f_temp							=	(pid->Out_Actual +10.0f)*150.0f;								//查看输出控制量
//	f_temp							=	(pos_in - m_motor_rt_para.m_abs_encoder.f_abs_pos_init + 5.0f)*400.0f;											//实际位置值
	u32_temp						=	(uint32_t)(f_temp);										
	if(u32_temp>4095) u32_temp 		=	4095;
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);
	
	f_pos_refin						=	(m_motor_ctrl.m_motion_ctrl.f_set_position + 5.0 )*400.0f;		//跟随目标幅值
	u32_pos_refin					=	(uint32_t)(f_pos_refin);
	if(u32_pos_refin>4095) u32_pos_refin 		=	4095;
	DAC_SetChannel2Data(DAC_Align_12b_R,(uint16_t)u32_pos_refin);
#endif
	
/*将pid计算值赋值到设置值中*/
	m_motor_ctrl.m_motion_ctrl.f_set_speed		=	pid_inc;
	
/*设置速度低于电机最大速度*/
	
	m_motor_ctrl.m_motion_ctrl.u8_speed_set_data_refreshed	=	1;
}

	/*---------------------------------------------------------------------------
	函数名称			：(void)
	参数含义			：null
	函数功能			：CAN发送电流速度位置值
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
	
	//CAN发送报文测试
	while(!CAN1_TX_Data((CanTxMsg*)&(m_can.TxMessage),
						CMD_GET_SPD_POS_CUR,
						(uint8_t*)&data,
						8));
	
}
	
	
	
	/*---------------------------------------------------------------------------
	函数名称			：(void)
	参数含义			：null
	函数功能			：PID计算
	----------------------------------------------------------------------------*/





/*********************************************END OF FILE**************************************************/

