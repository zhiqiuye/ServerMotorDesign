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

/* Private variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/

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
//	pid->D_Out				=	pid->Kd * (pid->Err_T_0 - 2.0f * pid->Err_T_1 + pid->Err_T_2);	//微分部分
	
	pid->pid_increment		=	pid->P_Out + pid->I_Out;// + pid->D_Out;							//计算PID增量
	
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
	uint16_t			Hall_Value;
	Hall_Value		=	Hall_State_Read();
	//根据获得的Hall状态设置tim1的PWM状态
	if(m_motor_ctrl.u8_dir == 0)											//设置电机正转
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
	else if(m_motor_ctrl.u8_dir == 1)										//设置电机反转
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
	函数名称			：PWM_TIM_Start(void)
	参数含义			：null
	函数功能			：开启定时器产生PWM
	----------------------------------------------------------------------------*/





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
	TIM_Cmd(TIM2,ENABLE);
}


	/*---------------------------------------------------------------------------
	函数名称			：CoilCurrentRefresh_TIM_Halt(void)
	参数含义			：null
	函数功能			：开启电流环的更新定时器中断
	----------------------------------------------------------------------------*/
void	CurrentLoopRefresh_TIM_Halt(void)
{
	TIM_Cmd(TIM2,DISABLE);
}

	/*---------------------------------------------------------------------------
	函数名称			：SpeedPosLoopRefresh_TIM_Start(void)
	参数含义			：null
	函数功能			：开启速度位置环的更新定时器中断
	----------------------------------------------------------------------------*/
void	SpeedPosLoopRefresh_TIM_Start(void)
{
	TIM_ClearFlag(TIM9,TIM_IT_Update);
	TIM_Cmd(TIM9,ENABLE);
}



	/*---------------------------------------------------------------------------
	函数名称			：SpeedPosLoopRefresh_TIM_Halt(void)
	参数含义			：null
	函数功能			：开启速度位置环的更新定时器中断
	----------------------------------------------------------------------------*/
void	SpeedPosLoopRefresh_TIM_Halt(void)
{
	TIM_Cmd(TIM9,DISABLE);
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
	
	for(i = 0 ; i < 8 ; i++)											//在实际电流为0的时候，每10ms取一次电流值压入buf
	{
		m_motor_rt_para.u16_u_curr_bias	+=	m_motor_rt_para.u16_u_current;
		m_motor_rt_para.u16_v_curr_bias	+=	m_motor_rt_para.u16_v_current;
		m_motor_rt_para.u16_w_curr_bias	+=	m_motor_rt_para.u16_w_current;
		OSTimeDlyHMSM(0,0,0,20);
	}

	m_motor_rt_para.u16_u_curr_bias		=	m_motor_rt_para.u16_u_curr_bias>>3;
	m_motor_rt_para.u16_v_curr_bias		=	m_motor_rt_para.u16_v_curr_bias>>3;
	m_motor_rt_para.u16_w_curr_bias		=	m_motor_rt_para.u16_w_curr_bias>>3;
	
	m_motor_rt_para.f_adc_U_I			=	0.0f;						//将错误更新的电流值归零，否则影响第一次电流环的PID计算
	m_motor_rt_para.f_adc_V_I			=	0.0f;
	m_motor_rt_para.f_adc_W_I			=	0.0f;
}


	/*---------------------------------------------------------------------------
	函数名称			：Curr_PID_Cal(volatile PID_Struct * pid)
	参数含义			：null
	函数功能			：电流环PID计算，电流环输入只有正值，仅进行PWM占空比的设置
							增量式PID计算
	----------------------------------------------------------------------------*/
void	Curr_PID_Cal(volatile PID_Struct * pid)
{
	uint32_t		ccr;
	float			f_temp;
	uint32_t		u32_temp;
	float			pid_inc = 0.0f;
	float			curr_in = 0.0f;
	
	/*判断输入电流值是否在额定电流内*/
	curr_in					=	m_motor_rt_para.f_adc_U_I;
	
	if(m_motor_rt_para.f_adc_U_I > m_motor.f_max_current)
		curr_in	 			=	m_motor.f_max_current;
	
	/*计算增量pid输出*/
	pid_inc					=	Increment_PID_Cal((PID_Struct*)pid,curr_in);			//暂时使用PI
	
	//------------test 20180808	
	//------------20180813  将E输出
	f_temp					=	m_current_pid.curr_pid.Feed_Back;							//跟踪目标电压
	u32_temp				=	(uint32_t)(f_temp * 1240.9f);
	
	if(u32_temp>4095) u32_temp = 4095;
	
	DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)u32_temp);
	
	//------------test 20180808
	
	/*将pid输出值转化到10-4190之间*/
	m_current_pid.PW		+=	pid_inc;
	
	/*对PID输出做出赋值限制*/
	if(m_current_pid.PW > MAX_DUTY_CYCLE)
		m_current_pid.PW		=	MAX_DUTY_CYCLE;
	else if(m_current_pid.PW < MIN_DUTY_CYCLE)
		m_current_pid.PW		=	MIN_DUTY_CYCLE;
	else
		;
	
	/*更改PWM占空比*/
	ccr						=	(int32_t)m_current_pid.PW;
	TIM1->CCR1				=	ccr;
	TIM1->CCR2				=	ccr;
	TIM1->CCR3				=	ccr;
	
//	ccr						=	(uint16_t)(m_motor_rt_para.f_adc_U_I * 1000.0f);

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
	
	
	
	
	
	/*---------------------------------------------------------------------------
	函数名称			：(void)
	参数含义			：null
	函数功能			：PID计算
	----------------------------------------------------------------------------*/





/*********************************************END OF FILE**************************************************/

