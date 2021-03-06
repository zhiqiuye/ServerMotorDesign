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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
//#define		MEASURE_CURRENT_REFIN
//#define		MEASURE_SPEED_REFIN
#define		MEASURE_POSITION_REFIN

/* Private function prototypes -----------------------------------------------*/
const void		NULL_Switch(void);
const void		H1_L2(void);
const void		H1_L3(void);
const void		H2_L1(void);
const void		H2_L3(void);
const void		H3_L2(void);
const void		H3_L1(void);

/* Private variables ---------------------------------------------------------*/
//霍尔换向表
const	void( *switch_table[2][8])() ={	{NULL_Switch,	H2_L3,		H1_L2,		H1_L3,		H3_L1,		H2_L1,		H3_L2,	NULL_Switch},
										{NULL_Switch,	H3_L2,		H2_L1,		H3_L1,		H1_L3,		H1_L2,		H2_L3,	NULL_Switch}}; 

//const	void	(*switch_table[2][8])()			=	{	{NULL_Switch,	CW_H2_L3,		CW_H1_L2,		CW_H1_L3,		CW_H3_L1,		CW_H2_L1,		CW_H3_L2,	NULL_Switch},
//														{NULL_Switch,	CCW_H3_L2,		CCW_H2_L1,		CCW_H3_L1,		CCW_H1_L3,		CCW_H1_L2,		CCW_H2_L3,	NULL_Switch}}; 
//	
//电流相序表
const	uint8_t		current_senser_table[2][7]	=	{{0, 2, 1, 2, 0, 0, 1},{0, 1, 0, 0, 2, 1, 2}};										


/* Private functions ---------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	函数名称			：Increment_PID_Cal(PID_Struct* pid)
	参数含义			：null
	函数功能			：增量PID计算过程，输出值在-10到10之间
							本地函数
	----------------------------------------------------------------------------*/
float	Increment_PID_Cal(PID_Struct* pid,float new_feedback)
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
float	Increment_PID_Cal_int(PID_Struct* pid, int32_t new_feedback)
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
	m_motor_rt_para.u8_hall_state		=	Hall_State_Read();								//记录hall状态
	
	switch_table[m_motor_ctrl.u8_dir][m_motor_rt_para.u8_hall_state]();						//根据方向以及hall相位进行换向，采用函数指针方式
}


	/*---------------------------------------------------------------------------
	函数名称			：NULL_Switch(void)
	参数含义			：null
	函数功能			：空
	----------------------------------------------------------------------------*/
const void	NULL_Switch(void)
{}

	/*---------------------------------------------------------------------------
	函数名称			：H1_L2(void)
	参数含义			：null
	函数功能			：H1和L2导通
	----------------------------------------------------------------------------*/
const void	H1_L2(void)
{
	TIM1_CH1_PWM_CH1N_OFF();
	TIM1_CH2_OFF_CH2N_PWM();
	TIM1_CH3_OFF_CH3N_OFF();
}


	/*---------------------------------------------------------------------------
	函数名称			：H1_L3(void)
	参数含义			：null
	函数功能			：H1和L3导通
	----------------------------------------------------------------------------*/
const void	H1_L3(void)
{
	TIM1_CH1_PWM_CH1N_OFF();
	TIM1_CH2_OFF_CH2N_OFF();
	TIM1_CH3_OFF_CH3N_PWM();
}


	/*---------------------------------------------------------------------------
	函数名称			：H2_L1(void)
	参数含义			：null
	函数功能			：H2和L1导通
	----------------------------------------------------------------------------*/
const void	H2_L1(void)
{
	TIM1_CH1_OFF_CH1N_PWM();
	TIM1_CH2_PWM_CH2N_OFF();
	TIM1_CH3_OFF_CH3N_OFF();
}


	/*---------------------------------------------------------------------------
	函数名称			：H2_L3(void)
	参数含义			：null
	函数功能			：H2和L3导通
	----------------------------------------------------------------------------*/
const void	H2_L3(void)
{
	TIM1_CH1_OFF_CH1N_OFF();
	TIM1_CH2_PWM_CH2N_OFF();
	TIM1_CH3_OFF_CH3N_PWM();
}


	/*---------------------------------------------------------------------------
	函数名称			：H3_L1(void)
	参数含义			：null
	函数功能			：H3和L1导通
	----------------------------------------------------------------------------*/
const void	H3_L1(void)
{
	TIM1_CH1_OFF_CH1N_PWM();
	TIM1_CH2_OFF_CH2N_OFF();
	TIM1_CH3_PWM_CH3N_OFF();
}


	/*---------------------------------------------------------------------------
	函数名称			：H3_L2(void)
	参数含义			：null
	函数功能			：H3和L2导通
	----------------------------------------------------------------------------*/
const void	H3_L2(void)
{
	TIM1_CH1_OFF_CH1N_OFF();
	TIM1_CH2_OFF_CH2N_PWM();
	TIM1_CH3_PWM_CH3N_OFF();
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
	m_motor_ctrl.u8_is_currloop_used	=	1;
	TIM_Cmd(TIM1,ENABLE);
}


	/*---------------------------------------------------------------------------
	函数名称			：CoilCurrentRefresh_TIM_Halt(void)
	参数含义			：null
	函数功能			：关闭电流环的更新定时器中断
	----------------------------------------------------------------------------*/
void	CurrentLoopRefresh_TIM_Halt(void)
{
	m_motor_ctrl.u8_is_currloop_used	=	0;
	TIM1->CCR1	=	10;
	TIM1->CCR2	=	10;
	TIM1->CCR3	=	10;
}

	/*---------------------------------------------------------------------------
	函数名称			：SpeedPosLoopRefresh_TIM_Start(void)
	参数含义			：null
	函数功能			：	开启速度环的更新定时器中断
							开启速度环必须开启电流环
	----------------------------------------------------------------------------*/
void	SpeedLoopRefresh_TIM_Start(void)
{
	m_motor_ctrl.u8_is_speedloop_used	=	1;
	m_motor_ctrl.u8_is_currloop_used	=	1;
	
	TIM_ClearFlag(TIM2,TIM_IT_Update);
	TIM_Cmd(TIM2,ENABLE);
}



	/*---------------------------------------------------------------------------
	函数名称			：SpeedPosLoopRefresh_TIM_Halt(void)
	参数含义			：null
	函数功能			：开启速度环的更新定时器中断
	----------------------------------------------------------------------------*/
void	SpeedLoopRefresh_TIM_Halt(void)
{
	m_motor_ctrl.u8_is_speedloop_used	=	0;
}



	/*---------------------------------------------------------------------------
	函数名称			：PositionLoopRefresh_TIM_Start(void)
	参数含义			：null
	函数功能			：开启位置环的更新定时器中断
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
	函数名称			：PositionLoopRefresh_TIM_Start(void)
	参数含义			：null
	函数功能			：关闭位置环的更新
	----------------------------------------------------------------------------*/
void	PositionLoopRefresh_TIM_Halt(void)
{
	m_motor_ctrl.u8_is_posloop_used		=	0;
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
	
	if(m_sys_state.u8_cur_state != Prepare_state)						//更改状态机状态
		m_sys_state.u8_cur_state = Prepare_state;
	
	CurrentLoopRefresh_TIM_Halt();										//暂停电流环更新
	
	TIM1->CCR1		=	10;												//将PWM占空比设为最低
	TIM1->CCR2		=	10;
	TIM1->CCR3		=	10;
	
	IR2130S_force_reset();												//占空比为0
	
	Current_Filter_Init();												//初始化电流滤波器
	
	m_motor_rt_para.u16_uvw_curr_bias		=	0x641;					//1.29V作为偏置

	m_motor_rt_para.f_adc_UVW_I				=	0.0f;					//将错误更新的电流值归零，否则影响第一次电流环的PID计算
}


	/*---------------------------------------------------------------------------
	函数名称			：Curr_PID_Cal(volatile PID_Struct * pid)
	参数含义			：null
	函数功能			：电流环PID计算，电流环输入只有正值，仅进行PWM占空比的设置
							增量式PID计算
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
	
	/*判断输入电流值是否在额定电流内*/
	curr_in					=	m_motor_rt_para.f_adc_UVW_I;
	
	if(curr_in > m_motor.f_max_current)
		curr_in	 			=	m_motor.f_max_current;
	
	/*计算增量pid输出*/
	pid_inc					=	Increment_PID_Cal((PID_Struct*)pid,curr_in);			//暂时使用PI

#ifdef	MEASURE_CURRENT_REFIN	
	//------------test 20180808	
	f_temp					=	m_current_pid.curr_pid.Feed_Back;						//跟踪目标电压
	u32_temp				=	(uint32_t)(f_temp * 1240.9f);
	if(u32_temp>4095) u32_temp = 4095;													//将数字量限定幅值
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);							//开启DA转换
#endif
	
	/*将pid输出值转化到10-4190之间*/
	m_current_pid.PW		=	pid_inc * 420.0f;
	
	/*对PID输出做出赋值限制*/
	if(m_current_pid.PW > MAX_DUTY_CYCLE)
		m_current_pid.PW	=	MAX_DUTY_CYCLE;
	else if(m_current_pid.PW < MIN_DUTY_CYCLE)
		m_current_pid.PW	=	MIN_DUTY_CYCLE;
	else
		;
	
	/*更改PWM占空比*/
	ccr						=	(uint16_t)m_current_pid.PW;
	TIM1->CCR1				=	ccr;
	TIM1->CCR2				=	ccr;
	TIM1->CCR3				=	ccr;
}




	/*---------------------------------------------------------------------------
	函数名称			：Speed_PID_Cal(volatile PID_Struct * pid)
	参数含义			：速度环pid结构体
	函数功能			：速度环的PID计算
	----------------------------------------------------------------------------*/
void	Speed_PID_Cal(volatile PID_Struct * pid)
{
	float	spd_in			=	0.0f;											//获取的速度值
	float	pid_inc			=	0.0f;
#ifdef	MEASURE_SPEED_REFIN
	uint32_t	u32_temp;
	float		f_temp;
#endif
	
	/*计算增量pid输出*/
	spd_in					=	m_motor_rt_para.f_motor_cal_speed;				//获取反馈速度值
	pid_inc					=	Increment_PID_Cal((PID_Struct*)pid,spd_in);

#ifdef	MEASURE_SPEED_REFIN
//--------------------20180817test	
	f_temp					=	spd_in + 1.5f;									//跟踪目标电压
	u32_temp				=	(uint32_t)(f_temp * 819.0f);					//5rps对应3.3V
	if(u32_temp>4095) u32_temp 		= 	4095;
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);
#endif
	
	/*将pid输出值累加到电流设置值*/
	m_motor_ctrl.f_set_current		=	pid_inc;		
	
	/*限定电流值为正，并且根据电流值正负来换向*/
	if(m_motor_ctrl.f_set_current > 0.1f)
	{
		m_motor_ctrl.u8_dir			=	1;
		switch_table[m_motor_ctrl.u8_dir][m_motor_rt_para.u8_hall_state]();
	}
	else if(m_motor_ctrl.f_set_current < -0.1f)
	{
		m_motor_ctrl.u8_dir			=	0;
		switch_table[m_motor_ctrl.u8_dir][m_motor_rt_para.u8_hall_state]();
	}
	else
	{
		m_motor_ctrl.f_set_current	=	0.0f;
	}
	
	m_motor_ctrl.u8_current_set_data_refreshed	=	1;							//电流设置数据更新
}

	/*---------------------------------------------------------------------------
	函数名称			：Position_PID_Cal(volatile PID_Struct * pid)
	参数含义			：位置环pid结构体
	函数功能			：位置环的PID计算
	----------------------------------------------------------------------------*/
void	Position_PID_Cal(volatile PID_Struct * pid)
{
	float		pos_in				=	0.0f;
	float		pid_inc				=	0.0f;

#ifdef	MEASURE_POSITION_REFIN
	uint32_t	u32_temp;
	float		f_temp;
#endif
/*计算增量pid输出*/
	pos_in							=	(float)m_motor_rt_para.i32_pulse_cnt;	//获取反馈位置值
	pid_inc							=	Increment_PID_Cal((PID_Struct*)pid,pos_in);

#ifdef	MEASURE_POSITION_REFIN
//--------------------20180820test	
	f_temp							=	pos_in + 2000.0f;//m_motor_ctrl.f_set_position	+ 1000.0f;//					//跟踪目标电压
	u32_temp						=	(uint32_t)(f_temp);						//将-1000cnts到1000cnts 投射到0-3.3V
	if(u32_temp>4095) u32_temp 		=	4095;
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);
#endif
	
/*将pid计算值赋值到设置值中*/
	m_motor_ctrl.f_set_speed		=	pid_inc;
	
/*设置速度低于电机最大速度*/
	
	m_motor_ctrl.u8_speed_set_data_refreshed	=	1;
}

	/*---------------------------------------------------------------------------
	函数名称			：(void)
	参数含义			：null
	函数功能			：PID计算
	----------------------------------------------------------------------------*/
	
	
	
	
	
	/*---------------------------------------------------------------------------
	函数名称			：(void)
	参数含义			：null
	函数功能			：PID计算
	----------------------------------------------------------------------------*/





/*********************************************END OF FILE**************************************************/

