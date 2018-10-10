/**
  ******************************************************************************
  * @file    Project/user/foc_reversal_svpwm.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170928
  * @brief   霍尔换向以及SVPWM操作
  ******************************************************************************
  ******************************************************************************
  */

/*
  FOC 控制框图                                                                                                         
                           ____                          ____            __________                 _________                            
   speed_ref              |    |  Iq_ref                |    |   Vq     |          |     V_alpha   |         |        PWM_a          ___________                 
      ------> + --------->| PI |---------> + ---------->| PI |--------->|          |-------------->|         |--------------------->|           |                                                                       
              ^           |____|           ^            |____|          | Reverse  |               |         |                      |           |
              |                            |                            |          |               |         |        PWM_b         |           |
              |                            |                            |          |               |  SVPWM  |--------------------->|   MOTOR   |          
              |                            |             ____ 	        |          |               |         | 						|			|		 
              |             Id_ref         |            |    |    Vd    |   Park   |     V_beta    |         |        PWM_c         |           | 
              |           ----------> + --------------->| PI |----------|          |-------------->|         |--------------------->|___________|                                                                  
              |                       ^    |            |____|          |__________|               |_________|                         | | | |    
              |                       |    |                                 ^                                                         | | | |           
              |                       |    |                                 |theta                                                    | | | |           
              |                       |    |                             __________                 _________       I_a                | | | |               
              |                       |    |      I_q                   |          |    I_alpha    |         |<------------------------+ | | |           
              |                       |    +<---------------------------|          |<--------------|         |      I_b                  | | |             
              |                       |           I_d                   |   Park   |    I_beta     |  Clark  |<--------------------------+ | |              
              |                       +<--------------------------------|          |<--------------|         |      I_c                    | |          
              |                                                         |__________|               |_________|<----------------------------+ |
              |                                                              ^                                                               |
              |                                                              |                                                               |
              |                                                              |theta                 __________                               |
              | omiga                                                        |                     |          |                              |
              |                                                              +---------------------|  Speed   |          encoder             |
              |                                                                                    |    &     |<-----------------------------+
              +<-----------------------------------------------------------------------------------| Position |                               
                                                                                                   |__________|                                         
                                                                                                                                             
                                                                                                                                             
        Clark变换：
					I_alpha = Ia
                    I_beta  = (Ia + 2*Ib)/(3^0.5)

        Park变换：
                    I_q	    = I_beta * cos(theta) - I_alpha * sin(theta)
                    I_d     = I_alpha * cos(theta) + I_beta * sin(theta)

        Reverse Park变换：
                    V_alpha = Vd * cos(theta) - Vq * sin(theta)
                    V_beta  = Vq * cos(theta) + Vd * sin(theta)
		                                                                                                                                     			
		                                                                                                                                     
		                              (010)        II         (110)                                                                                    
		                               U_2  _________________  U_6                                                                               																																			 
		                                   /\     beta      /\                                                                                 																																			 
		                                  /  \      |      /  \                                                                                																																			 
		                                 /    \     |     /    \                                                                               																																			 
		                           III  /      \    |    /      \  I                                                                           																																			 
			                           /        \   |   /        \                                                                                 																																			 
			                          /          \  |  /          \                                                                                																																			 
			                         /            \ | /            \                                                                                																																			 
			                        /     U_7      \|/     U_0      \                                                                              																																			 
			         (011)   U_3   -----------------+----------------- U_4  (100)    alpha                                                                                                              																																			 
			                        \    (111)     / \    (000)     /                                                                               																																			 
			                         \            /   \            /                                                                                																																			 
			                          \          /     \          /                                                                                 																																			 
			                           \        /       \        /                                                                                  																																			 
			                       IV   \      /         \      /   VI                                                                              																																			 
			                             \    /           \    /                                                                                    																																			 
			                              \  /             \  /                                                                                     																																			 
			                               \/_______________\/                                                                                      																																			 
			                          U_1                       U_5                                                                                      																																			 
			                         (100)          V          (101)                                                                                      																																			 
		                                                                                                                                     																																			 


*/

#include	"stm32f4xx.h"
#include	"stm32f4xx_gpio.h"
#include	"stm32f4xx_tim.h"
#include	"global_parameters.h"
#include	"foc_reversal_svpwm.h"
#include	"arm_math.h"


#define		DEG_2_RAD			0.0174532925199f

uint16_t	rotor_static_tim = 0;

//以下8个函数只是SVPWM时的换向函数，确定哪些开关管导通，哪些开关管关闭
//占空比需要在单独计算
//////////////////////////////////////////////////////////////////////////////////
	/*---------------------------------------------------------------------------
	函数名称			：	U_OFF_V_OFF_W_OFF(void)
	参数含义			：	null
	函数功能			：	H桥开关管状态			000，
						上桥臂a/b/c都关闭
						下桥臂a/b/c都开启
	----------------------------------------------------------------------------*/
void	U_OFF_V_OFF_W_OFF(void)
{
//	ch1 off-pwm
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1为PWM2模式
//	TIM1->CCER &=0xFFFE;			//CCER->CC1E	=	0
//	TIM1->CCER |=0x0004;			//CCER->CC1NE	=	1
//	ch2 off-pwm
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2为PWM2模式
//	TIM1->CCER &=0xFFEF;			//CCER->CC2E	=	0
//	TIM1->CCER |=0x0040;			//CCER->CC2NE	=	1
//	ch3 off-pwm
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3为PWM2模式
//	TIM1->CCER &=0xFEFF;			//CCER->CC3E	=	0
//	TIM1->CCER |=0x0400;			//CCER->CC3NE	=	1
	
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2为PWM2模式
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3为PWM2模式
	TIM1->CCER	&=	0xFEEE;			//CCER->CC1/2/3E	=	0
	TIM1->CCER	|=	0x0444;			//CCER->CC1/2/3NE	=	1
}


	/*---------------------------------------------------------------------------
	函数名称			：	U_ON_V_OFF_W_OFF(void)
	参数含义			：	null
	函数功能			：	H桥开关管状态			100，
						上桥臂a开启，b/c都关闭
						下桥臂a关闭，b/c都开启
	----------------------------------------------------------------------------*/
void	U_ON_V_OFF_W_OFF(void)
{
//	//ch1 pwm-off
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1为PWM2模式
//	TIM1->CCER |=0x0001;			//CCER->CC1E	=	1
//	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	=	0
//	//ch2 off-pwm
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2为PWM2模式
//	TIM1->CCER &=0xFFEF;			//CCER->CC2E	=	0
//	TIM1->CCER |=0x0040;			//CCER->CC2NE	=	1
//	//ch3 off-pwm
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3为PWM2模式
//	TIM1->CCER &=0xFEFF;			//CCER->CC3E	=	0
//	TIM1->CCER |=0x0400;			//CCER->CC3NE	=	1
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2为PWM2模式
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3为PWM2模式
	TIM1->CCER	&=	0xFEEB;
	TIM1->CCER	|=	0x0441;
}


	/*---------------------------------------------------------------------------
	函数名称			：	U_OFF_V_ON_W_OFF(void)
	参数含义			：	null
	函数功能			：	H桥开关管状态			010，
						上桥臂b开启，a/c都关闭
						下桥臂b关闭，a/c都开启
	----------------------------------------------------------------------------*/
void	U_OFF_V_ON_W_OFF(void)
{
//	//ch1 off-pwm
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1为PWM2模式
//	TIM1->CCER &=0xFFFE;			//CCER->CC1E	=	0
//	TIM1->CCER |=0x0004;			//CCER->CC1NE	=	1
//	//ch2 pwm-off
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2为PWM2模式
//	TIM1->CCER |=0x0010;			//CCER->CC2E	=	1
//	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	=	0
//	//ch3 off-pwm
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3为PWM2模式
//	TIM1->CCER &=0xFEFF;			//CCER->CC3E	=	0
//	TIM1->CCER |=0x0400;			//CCER->CC3NE	=	1
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2为PWM2模式
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3为PWM2模式
	TIM1->CCER	&=	0xFEBE;			//CCER->CC1/3E	=	0		CCER->CC2E	=	1
	TIM1->CCER	|=	0x0414;			//CCER->CC1/3NE	=	1		CCER->CC2NE	=	0
}


	/*---------------------------------------------------------------------------
	函数名称			：	U_ON_V_ON_W_OFF(void)
	参数含义			：	null
	函数功能			：	H桥开关管状态			110，
						上桥臂a/b开启，c关闭
						下桥臂a/b关闭，c开启
	----------------------------------------------------------------------------*/
void	U_ON_V_ON_W_OFF(void)
{
	//ch1 pwm-off
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1为PWM2模式
//	TIM1->CCER |=0x0001;			//CCER->CC1E	=	1
//	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	=	0
	//ch2 pwm-off
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2为PWM2模式
//	TIM1->CCER |=0x0010;			//CCER->CC2E	=	1
//	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	=	0	
	//ch3 off-pwm
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3为PWM2模式
//	TIM1->CCER &=0xFEFF;			//CCER->CC3E	=	0
//	TIM1->CCER |=0x0400;			//CCER->CC3NE	=	1
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2为PWM2模式
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3为PWM2模式	
	TIM1->CCER 	|=	0x0411;			//CCER->CC1/2E	=	1	CCER->CC3NE	=	1
	TIM1->CCER 	&=	0xFEBB;			//CCER->CC1/2E	=	1	CCER->CC3E	=	0
}


	/*---------------------------------------------------------------------------
	函数名称			：	U_OFF_V_OFF_W_ON(void)
	参数含义			：	null
	函数功能			：	H桥开关管状态			001，
						上桥臂c开启，a/b关闭
						下桥臂c关闭，a/b开启
	----------------------------------------------------------------------------*/
void	U_OFF_V_OFF_W_ON(void)
{
//	//ch1 off-pwm
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1为PWM2模式
//	TIM1->CCER &=0xFFFE;			//CCER->CC1E	=	0
//	TIM1->CCER |=0x0004;			//CCER->CC1NE	=	1
//	//ch2 off-pwm
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2为PWM2模式
//	TIM1->CCER &=0xFFEF;			//CCER->CC2E	=	0
//	TIM1->CCER |=0x0040;			//CCER->CC2NE	=	1
//	//ch3 pwm-off
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3为PWM2模式
//	TIM1->CCER |=0x0100;			//CCER->CC3E	=	1
//	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	=	0	
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2为PWM2模式
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3为PWM2模式	
	TIM1->CCER 	&=	0xFBEE;			//CCER->CC1/2E	=	0		CCER->CC3NE	=	0
	TIM1->CCER 	|=	0x0144;			//CCER->CC1/2NE	=	1		CCER->CC3NE	=	0
}


	/*---------------------------------------------------------------------------
	函数名称			：	U_ON_V_OFF_W_ON(void)
	参数含义			：	null
	函数功能			：	H桥开关管状态			101，
						上桥臂a/c开启，b关闭
						下桥臂a/c关闭，b开启
	----------------------------------------------------------------------------*/
void	U_ON_V_OFF_W_ON(void)
{
//	//ch1 pwm-off
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1为PWM2模式
//	TIM1->CCER |=0x0001;			//CCER->CC1E	=	1
//	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	=	0	
//	//ch2 off-pwm
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2为PWM2模式
//	TIM1->CCER &=0xFFEF;			//CCER->CC2E	=	0
//	TIM1->CCER |=0x0040;			//CCER->CC2NE	=	1
//	//ch3 pwm-off
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3为PWM2模式
//	TIM1->CCER |=0x0100;			//CCER->CC3E	=	1
//	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	=	0
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2为PWM2模式
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3为PWM2模式
	TIM1->CCER 	|=	0x0141;			//CCER->CC1/3E	=	1		CCER->CC2E	=	0
	TIM1->CCER	&=	0xFBEB;			//CCER->CC1/3NE	=	0		CCER->CC2NE	=	1
}


	/*---------------------------------------------------------------------------
	函数名称			：	U_OFF_V_ON_W_ON(void)
	参数含义			：	null
	函数功能			：	H桥开关管状态			011，
						上桥臂b/c开启，a关闭
						下桥臂b/c关闭，a开启
	----------------------------------------------------------------------------*/
void	U_OFF_V_ON_W_ON(void)
{
//	//ch1 off-pwm
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1为PWM2模式
//	TIM1->CCER &=0xFFFE;			//CCER->CC1E	=	0
//	TIM1->CCER |=0x0004;			//CCER->CC1NE	=	1
//	//ch2 pwm-off
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2为PWM2模式
//	TIM1->CCER |=0x0010;			//CCER->CC2E	=	1
//	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	=	0
//	//ch3 pwm-off
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3为PWM2模式
//	TIM1->CCER |=0x0100;			//CCER->CC3E	=	1
//	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	=	0
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2为PWM2模式
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3为PWM2模式
	TIM1->CCER	&=	0xFBBE;			//CCER->CC2/3NE	=	0	CCER->CC1E	=	0
	TIM1->CCER 	|=	0x0114;			//CCER->CC2/3E	=	1	CCER->CC1NE	=	1
}


	/*---------------------------------------------------------------------------
	函数名称			：	U_ON_V_ON_W_ON(void)
	参数含义			：	null
	函数功能			：	H桥开关管状态			111，
						上桥臂a/b/c都开启
						下桥臂a/b/c都关闭
	----------------------------------------------------------------------------*/
void	U_ON_V_ON_W_ON(void)
{
//	//ch1 pwm-off
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1为PWM2模式
//	TIM1->CCER |=0x0001;			//CCER->CC1E	=	1
//	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	=	0	
//	//ch2 pwm-off
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2为PWM2模式
//	TIM1->CCER |=0x0010;			//CCER->CC2E	=	1
//	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	=	0	
//	//ch3 pwm-off
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3为PWM2模式
//	TIM1->CCER |=0x0100;			//CCER->CC3E	=	1
//	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	=	0

	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2为PWM2模式
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3为PWM2模式
	TIM1->CCER 	|=	0x0111;			//CCER->CC1/2/3E	=	1
	TIM1->CCER	&=	0xFBBB;			//CCER->CC1/2/3NE	=	0
}

//////////////////////////////////////////////////////////////////////////////////
	/*---------------------------------------------------------------------------
	函数名称			：	Increment_PI_Cal(PID_Struct* pid)
	参数含义			：	null
	函数功能			：	增量PI计算过程，
						本地函数
						输出值作为V_alpha和V_beta，最大为3^0.5/3=0.57735Udc=13.856f
	----------------------------------------------------------------------------*/
float	Increment_PI_Cal(PID_struct* pid,float new_feedback)
{
	pid->Feed_Back			=	new_feedback;													//更新反馈值
	pid->Err_T_1			=	pid->Err_T_0;													//更新T-1时刻误差
	pid->Err_T_0			=	pid->Ref_In - pid->Feed_Back;									//计算新的误差
	
	pid->P_Out				=	pid->Kp * (pid->Err_T_0 - pid->Err_T_1);						//比例部分
	pid->I_Out				=	pid->Ki * pid->Err_T_0;											//积分部分
	
	pid->pid_increment		=	pid->P_Out + pid->I_Out;										//计算PI增量
	
	pid->Out_Actual			=	pid->Out_Pre + pid->pid_increment;
	
	if(pid->Out_Actual > 13.856f)
		pid->Out_Actual		=	13.856f;
	if(pid->Out_Actual < -13.856f)
		pid->Out_Actual		=	-13.856f;
	
	pid->Out_Pre			=	pid->Out_Actual;
	
	return	pid->Out_Actual;
}






//////////////////////////////////////////////////////////////////////////////////
	/*---------------------------------------------------------------------------
	函数名称			：	ClarkConvert(void)
	参数含义			：	null
	函数功能			：	由电流环三相电流转化为alpha/beta坐标系下的电流值
						Clark变换：
									I_alpha = Ia
									I_beta  = (Ia + 2*Ib)/(3^0.5)
	----------------------------------------------------------------------------*/
void	ClarkConvert(void)
{
	m_motor_rt_para.m_clark.f_i_alpha		=	m_motor_rt_para.m_current_sensor.f_adc_U_I;
	m_motor_rt_para.m_clark.f_i_beta		=	0.577350269f * m_motor_rt_para.m_current_sensor.f_adc_U_I + 1.15470053837f * m_motor_rt_para.m_current_sensor.f_adc_V_I;
}


	/*---------------------------------------------------------------------------
	函数名称			：	ParkConvert(void)
	参数含义			：	null
	函数功能			：	由alpha/beta坐标系下电流转化为d/q坐标系下
						Park变换：
									I_q	    = I_beta * cos(theta) - I_alpha * sin(theta)
									I_d     = I_alpha * cos(theta) + I_beta * sin(theta)

						需要获取最新的转子电角度
	----------------------------------------------------------------------------*/
void	ParkConvert(void)
{
	float	theta_rad;
	
	theta_rad							=	m_motor_rt_para.m_park.f_theta_rad + m_motor_attribute_para.m_motor_ind_att.f_delta_theta_0 ;					//转子电角度转化为弧度，用于计算正余弦
	m_motor_rt_para.m_park.f_sin_theta	=	arm_sin_f32(theta_rad);
	m_motor_rt_para.m_park.f_cos_theta	=	arm_cos_f32(theta_rad);
	
	m_motor_rt_para.m_park.f_i_q		=	m_motor_rt_para.m_clark.f_i_beta * m_motor_rt_para.m_park.f_cos_theta
											- m_motor_rt_para.m_clark.f_i_alpha * m_motor_rt_para.m_park.f_sin_theta;
	
	m_motor_rt_para.m_park.f_i_d		=	m_motor_rt_para.m_clark.f_i_alpha * m_motor_rt_para.m_park.f_cos_theta
											+ m_motor_rt_para.m_clark.f_i_beta * m_motor_rt_para.m_park.f_sin_theta;
}



	/*---------------------------------------------------------------------------
	函数名称			：	RotorDetect(void)
	参数含义			：	null
	函数功能			：	更新转子位置信息，theta值
						用高速端编码器数据更新
						
						需要在Hall捕获中断中矫正
	----------------------------------------------------------------------------*/
void	RotorDetect(void)
{
	int16_t	temp_delta;
	
	m_motor_rt_para.m_park.u16_last_enc_cnts	=	m_motor_rt_para.m_park.u16_curt_enc_cnts;
	m_motor_rt_para.m_park.u16_curt_enc_cnts	=	TIM3->CNT;
	
	temp_delta	=	m_motor_rt_para.m_park.u16_curt_enc_cnts - m_motor_rt_para.m_park.u16_last_enc_cnts;
	
//	m_motor_rt_para.m_park.f_theta_rad			-=	((float)((int16_t)temp_delta) * m_motor_attribute_para.m_motor_ind_att.f_rad_per_cnts);
}




	/*---------------------------------------------------------------------------
	函数名称			：	RotorCorrection(void)
	参数含义			：	null
	函数功能			：	更新转子位置信息，theta值
						在增量编码器I相捕获中断中进行矫正处理

	----------------------------------------------------------------------------*/
void	RotorCorrection(void)
{
	m_motor_rt_para.m_park.f_theta_rad		=	m_motor_attribute_para.m_motor_ind_att.f_delta_theta_0;
	
	
	
//	if(m_motor_ctrl.m_motion_ctrl.u8_dir == 0)							//正转
//	{
//		switch (hall_state)
//		{
//			case	5:													//0-60deg
//				m_motor_rt_para.m_park.f_theta_rad	=	0.0f;
//				break;
//			case	4:													//60-120deg
//				m_motor_rt_para.m_park.f_theta_rad	=	1.047197f;
//				break;
//			case	6:													//120-180deg
//				m_motor_rt_para.m_park.f_theta_rad	=	2.094395f;
//				break;
//			case	2:													//180-240deg
//				m_motor_rt_para.m_park.f_theta_rad	=	3.141593f;
//				break;
//			case	3:													//240-300deg
//				m_motor_rt_para.m_park.f_theta_rad	=	4.1887902f;
//				break;
//			case	1:													//300-360deg
//				m_motor_rt_para.m_park.f_theta_rad	=	5.23598775f;
//				break;
//			default:
//				//
//				break;
//		}	
//	}
//	else																//反转
//	{
//		switch (hall_state)
//		{
//			case	2:													//0-60deg
//				m_motor_rt_para.m_park.f_theta_rad	=	1.047197f;
//				break;
//			case	3:													//60-120deg
//				m_motor_rt_para.m_park.f_theta_rad	=	2.094395f;
//				break;
//			case	1:													//120-180deg
//				m_motor_rt_para.m_park.f_theta_rad	=	3.141593f;
//				break;
//			case	5:													//180-240deg
//				m_motor_rt_para.m_park.f_theta_rad	=	4.1887902f;
//				break;
//			case	4:													//240-300deg
//				m_motor_rt_para.m_park.f_theta_rad	=	5.23598775f;
//				break;
//			case	6:													//300-360deg
//				m_motor_rt_para.m_park.f_theta_rad	=	0.0f;
//				break;
//			default:
//				//
//				break;
//		}		
//	}
}


	/*---------------------------------------------------------------------------
	函数名称			：	RotorRecognition(uint8_t hall_state)
	参数含义			：	null
	函数功能			：	更新转子位置信息，theta值
						刚启动时转子的角度估计，根据Hall状态计算
	----------------------------------------------------------------------------*/
void	RotorRecognition(uint8_t hall_state)
{
	if(m_motor_ctrl.m_motion_ctrl.u8_dir == 0)	//顺时针转
	{
		switch (hall_state)
		{
			case	4:													//0-60deg		认为是90deg
				m_motor_rt_para.m_park.f_theta_rad	=	1.5707963267948966192313216916398f;
				break;
			case	6:													//60-120deg		认为是150deg
				m_motor_rt_para.m_park.f_theta_rad	=	2.6179938779914943653855361527329f;
				break;
			case	2:													//120-180deg	认为是210deg
				m_motor_rt_para.m_park.f_theta_rad	=	3.6651914291880921115397506138261f;
				break;
			case	3:													//180-240deg	认为是270deg
				m_motor_rt_para.m_park.f_theta_rad	=	4.7123889803846898576939650749193f;
				break;
			case	1:													//240-300deg	认为是330deg
				m_motor_rt_para.m_park.f_theta_rad	=	5.7595865315812876038481795360124f;
				break;
			case	5:													//300-360deg	认为是30deg
				m_motor_rt_para.m_park.f_theta_rad	=	0.52359877559829887307710723054658f;
				break;
			default:
				//hall状态错误
				break;
		}	
	}
	else
	{
		
	}
}



	/*---------------------------------------------------------------------------
	函数名称			：	ReverseParkConvert(void)
	参数含义			：	null
	函数功能			：	由/q坐标系下电流转化为alpha/betad坐标系下
						Reverse Park变换：
									V_alpha = Vd * cos(theta) - Vq * sin(theta)
									V_beta  = Vq * cos(theta) + Vd * sin(theta)
	----------------------------------------------------------------------------*/
void	ReverseParkConvert(void)
{
	m_motor_rt_para.m_revpark.f_v_alpha	=	m_motor_rt_para.m_revpark.f_v_d * m_motor_rt_para.m_park.f_cos_theta
											- m_motor_rt_para.m_revpark.f_v_q * m_motor_rt_para.m_park.f_sin_theta;
	m_motor_rt_para.m_revpark.f_v_beta	=	m_motor_rt_para.m_revpark.f_v_q * m_motor_rt_para.m_park.f_cos_theta
											+ m_motor_rt_para.m_revpark.f_v_d * m_motor_rt_para.m_park.f_sin_theta;
}


	/*---------------------------------------------------------------------------
	函数名称			：	CheckSector(void)
	参数含义			：	null
	函数功能			：	计算参考电压所在扇区
						参考电压应该超前转子一定角度，因此不能用霍尔状态来进行相位判断
	----------------------------------------------------------------------------*/
void	CheckSector(void)
{
	float	temp_u1,temp_u2,temp_u3,half_ub,x_ua;
	float	t0,t1,t2,t3,t4,t5,t6,t7;
	float	Taon,Tbon,Tcon;
	
	temp_u1	=	m_motor_rt_para.m_revpark.f_v_beta;
	half_ub	=	m_motor_rt_para.m_revpark.f_v_beta * 0.5f;
	x_ua	=	m_motor_rt_para.m_revpark.f_v_alpha * 0.8660254f;
	temp_u2	=	x_ua - half_ub;
	temp_u3	=	-x_ua - half_ub;
	
	m_motor_rt_para.m_revpark.u8_vref_sector	=	0x00;
	
	if(temp_u1 > 0.00000001f)	m_motor_rt_para.m_revpark.u8_vref_sector += 1;
	
	if(temp_u2 > 0.00000001f)	m_motor_rt_para.m_revpark.u8_vref_sector += 2;
	
	if(temp_u3 > 0.00000001f)	m_motor_rt_para.m_revpark.u8_vref_sector += 4;
	
	switch (m_motor_rt_para.m_revpark.u8_vref_sector)
	{
		case 3:				//扇区号1
			t4	=	m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc * temp_u2;
			t6	=	m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc * temp_u1;
			t0	=	(PWM_CYCLE_F - t4 - t6)*0.5f;
			t7	=	t0;
			Taon=	(PWM_CYCLE_F - t4 - t6)*0.25f;
			Tbon=	Taon + t4 * 0.5f;
			Tcon=	Tbon + t6 * 0.5f;
			m_motor_rt_para.m_reverse.i16_T_1	=	(int16_t)Taon;
			m_motor_rt_para.m_reverse.i16_T_2	=	(int16_t)Tbon;
			m_motor_rt_para.m_reverse.i16_T_3	=	(int16_t)Tcon;
			break;
		case 1:				//扇区号2
			t2	=	-m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc * temp_u2;
			t6	=	-m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc * temp_u3;
			t0	=	(PWM_CYCLE_F - t2 - t6)*0.5f;
			t7	=	t0;
			Taon=	(PWM_CYCLE_F - t2 - t6)*0.25f;
			Tbon=	Taon + t2 * 0.5f;
			Tcon=	Tbon + t6 * 0.5f;
			m_motor_rt_para.m_reverse.i16_T_1	=	(uint16_t)Tbon;
			m_motor_rt_para.m_reverse.i16_T_2	=	(uint16_t)Taon;
			m_motor_rt_para.m_reverse.i16_T_3	=	(uint16_t)Tcon;
			break;
		
		case 5:				//扇区号3
			t2	=	m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc * temp_u1;
			t3	=	m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc * temp_u3;
			t0	=	(PWM_CYCLE_F - t2 - t3)*0.5f;
			t7	=	t0;
			Taon=	(PWM_CYCLE_F - t2 - t3)*0.25f;
			Tbon=	Taon + t2 * 0.5f;
			Tcon=	Tbon + t3 * 0.5f;
			m_motor_rt_para.m_reverse.i16_T_1	=	(int16_t)Tcon;
			m_motor_rt_para.m_reverse.i16_T_2	=	(int16_t)Taon;
			m_motor_rt_para.m_reverse.i16_T_3	=	(int16_t)Tbon;
			break;
		case 4:				//扇区号4
			t1	=	-m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc * temp_u1;
			t3	=	-m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc * temp_u2;
			t0	=	(PWM_CYCLE_F - t1 - t3)*0.5f;
			t7	=	t0;
			Taon=	(PWM_CYCLE_F - t1 - t3)*0.25f;
			Tbon=	Taon + t1 * 0.5f;
			Tcon=	Tbon + t3 * 0.5f;
			m_motor_rt_para.m_reverse.i16_T_1	=	(int16_t)Tcon;
			m_motor_rt_para.m_reverse.i16_T_2	=	(int16_t)Tbon;
			m_motor_rt_para.m_reverse.i16_T_3	=	(int16_t)Taon;
			break;

		case 6:				//扇区号5
			t1	=	m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc * temp_u3;
			t5	=	m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc * temp_u2;
			t0	=	(PWM_CYCLE_F - t1 - t5)*0.5f;
			t7	=	t0;
			Taon=	(PWM_CYCLE_F - t1 - t5)*0.25f;
			Tbon=	Taon + t1 * 0.5f;
			Tcon=	Tbon + t5 * 0.5f;
			m_motor_rt_para.m_reverse.i16_T_1	=	(int16_t)Tbon;
			m_motor_rt_para.m_reverse.i16_T_2	=	(int16_t)Tcon;
			m_motor_rt_para.m_reverse.i16_T_3	=	(int16_t)Taon;
			break;
		
		case 2:				//扇区号6
			t4	=	-m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc * temp_u3;
			t5	=	-m_motor_attribute_para.m_motor_ind_att.f_root3_Ts_Udc * temp_u1;
			t0	=	(PWM_CYCLE_F - t4 - t5)*0.5f;
			t7	=	t0;
			Taon=	(PWM_CYCLE_F - t4 - t5)*0.25f;
			Tbon=	Taon + t4 * 0.5f;
			Tcon=	Tbon + t5 * 0.5f;
			m_motor_rt_para.m_reverse.i16_T_1	=	(int16_t)Taon;
			m_motor_rt_para.m_reverse.i16_T_2	=	(int16_t)Tcon;
			m_motor_rt_para.m_reverse.i16_T_3	=	(int16_t)Tbon;
			break;

		default:
			m_motor_rt_para.m_reverse.i16_T_1	=	MIN_DUTY_CYCLE_INT16;
			m_motor_rt_para.m_reverse.i16_T_2	=	MIN_DUTY_CYCLE_INT16;
			m_motor_rt_para.m_reverse.i16_T_3	=	MIN_DUTY_CYCLE_INT16;
			break;
	}
}


	/*---------------------------------------------------------------------------
	函数名称			：	IsRotorStatic(void)
	参数含义			：	null
	函数功能			：	判断电机转子是否静止
	----------------------------------------------------------------------------*/
void	IsRotorStatic(void)
{
	if(m_motor_ctrl.m_motion_ctrl.u8_speed_read_data_refreshed		==	1)						//编码器数据有刷新
	{
		if(m_motor_ctrl.m_motion_ctrl.u8_foc_state					==	foc_init_state_0)		//处在FOC初始化0阶段
		{
			if(m_motor_rt_para.m_inc_encoder.u16_encoder_last_read	==	m_motor_rt_para.m_inc_encoder.u16_encoder_curr_read)
			{
				rotor_static_tim ++;
			}
			else
			{
				rotor_static_tim = 0;
			}
			if(rotor_static_tim >= 1000)
			{
				m_motor_ctrl.m_motion_ctrl.u8_foc_state				=	foc_init_state_1;
				rotor_static_tim									=	0;
			}
		}
		if(m_motor_ctrl.m_motion_ctrl.u8_foc_state					==	foc_init_state_2)
		{
			if(m_motor_rt_para.m_inc_encoder.u16_encoder_last_read	==	m_motor_rt_para.m_inc_encoder.u16_encoder_curr_read)
			{
				rotor_static_tim ++;
			}
			else
			{
				rotor_static_tim = 0;
			}
			if(rotor_static_tim >= 1000)
			{
				m_motor_ctrl.m_motion_ctrl.u8_foc_state				=	foc_init_state_1;
				m_motor_attribute_para.m_motor_ind_att.i32_cnts_AB	=	m_motor_rt_para.m_inc_encoder.i32_pulse_cnt - m_motor_attribute_para.m_motor_ind_att.i32_cnts_AB;
				m_motor_attribute_para.m_motor_ind_att.f_delta_theta_0	=	((float)m_motor_attribute_para.m_motor_ind_att.i32_cnts_AB)*m_motor_attribute_para.m_motor_ind_att.f_rad_per_cnts;
//				m_motor_ctrl.m_motion_ctrl.u8_foc_state					=	foc_init_over;
			}
		}
	}
}


	/*---------------------------------------------------------------------------
	函数名称			：	FOC_Cal(void)
	参数含义			：	null
	函数功能			：	FOC计算
	----------------------------------------------------------------------------*/
void	FOC_Cal(void)
{
	/*根据foc初始化状态进行相应操作*/
	switch (m_motor_ctrl.m_motion_ctrl.u8_foc_state)
	{
		case foc_init_state_0:										/**< foc给定电角度为0，等待电机转子稳定*/
			m_motor_rt_para.m_revpark.f_v_q		=	10.5f;
			m_motor_rt_para.m_revpark.f_v_d		=	0.0f;
			m_motor_rt_para.m_park.f_cos_theta	=	arm_cos_f32(m_motor_rt_para.m_park.f_theta_rad);
			m_motor_rt_para.m_park.f_sin_theta	=	arm_sin_f32(m_motor_rt_para.m_park.f_theta_rad);
			IsRotorStatic();										//检查转子是否静止，静止就进入下个状态
			break;
		
		case foc_init_state_1:										/**< 上一步电机转子稳定后，使用小电流，对转子进行旋转*/
			m_motor_rt_para.m_revpark.f_v_q		=	6.5f;
			m_motor_rt_para.m_revpark.f_v_d		=	0.0f;
			m_motor_rt_para.m_park.f_theta_rad	+= 0.0003141f;
			if(m_motor_rt_para.m_park.f_theta_rad >= 6.283185307179586476925286766559f)
				m_motor_rt_para.m_park.f_theta_rad = 0.0f;
			m_motor_rt_para.m_park.f_cos_theta	=	arm_cos_f32(m_motor_rt_para.m_park.f_theta_rad);
			m_motor_rt_para.m_park.f_sin_theta	=	arm_sin_f32(m_motor_rt_para.m_park.f_theta_rad);		//进入TIM3_ch3捕获中断后开启下一个步骤
			break;
		
		case foc_init_state_2:										/**< 转子重新转到0电位角*/
			m_motor_rt_para.m_revpark.f_v_q		=	6.5f;
			m_motor_rt_para.m_revpark.f_v_d		=	0.0f;
			m_motor_rt_para.m_park.f_cos_theta	=	arm_cos_f32(m_motor_rt_para.m_park.f_theta_rad);
			m_motor_rt_para.m_park.f_sin_theta	=	arm_sin_f32(m_motor_rt_para.m_park.f_theta_rad);
			IsRotorStatic();										//检查转子是否静止，静止就进入下个状态
			break;
		
		case foc_init_over:											/**< FOC准备完毕*/
			/*clark变换*/
			ClarkConvert();
			/*park变换*/
			ParkConvert();
			/*更新theta值*/
			RotorDetect();
			/*电流环PI计算*/
			m_pid.iq.Ref_In						=	m_motor_ctrl.m_motion_ctrl.f_set_iq;
			m_pid.id.Ref_In						=	m_motor_ctrl.m_motion_ctrl.f_set_id;
			m_motor_rt_para.m_revpark.f_v_q		=	Increment_PI_Cal(&(m_pid.iq),m_motor_rt_para.m_park.f_i_q);
			m_motor_rt_para.m_revpark.f_v_d		=	Increment_PI_Cal(&(m_pid.id),m_motor_rt_para.m_park.f_i_d);
			break;
		
		default:
			break;
	}
	
	/*park逆变换*/
	ReverseParkConvert();
	CheckSector();
	
	/*对PID输出做出赋值限制*/
	if(m_motor_rt_para.m_reverse.i16_T_1 > MAX_DUTY_CYCLE_INT16)
		m_motor_rt_para.m_reverse.i16_T_1	=	MAX_DUTY_CYCLE_INT16;
	if(m_motor_rt_para.m_reverse.i16_T_2 > MAX_DUTY_CYCLE_INT16)
		m_motor_rt_para.m_reverse.i16_T_2	=	MAX_DUTY_CYCLE_INT16;
	if(m_motor_rt_para.m_reverse.i16_T_3 > MAX_DUTY_CYCLE_INT16)
		m_motor_rt_para.m_reverse.i16_T_3	=	MAX_DUTY_CYCLE_INT16;
	if(m_motor_rt_para.m_reverse.i16_T_1 < MIN_DUTY_CYCLE_INT16)
		m_motor_rt_para.m_reverse.i16_T_1	=	MIN_DUTY_CYCLE_INT16;
	if(m_motor_rt_para.m_reverse.i16_T_2 < MIN_DUTY_CYCLE_INT16)
		m_motor_rt_para.m_reverse.i16_T_2	=	MIN_DUTY_CYCLE_INT16;
	if(m_motor_rt_para.m_reverse.i16_T_3 < MIN_DUTY_CYCLE_INT16)
		m_motor_rt_para.m_reverse.i16_T_3	=	MIN_DUTY_CYCLE_INT16;
	
	/*更改PWM占空比*/
	TIM1->CCR1				=	4200 - (uint16_t)m_motor_rt_para.m_reverse.i16_T_1;
	TIM1->CCR2				=	4200 - (uint16_t)m_motor_rt_para.m_reverse.i16_T_2;
	TIM1->CCR3				=	4200 - (uint16_t)m_motor_rt_para.m_reverse.i16_T_3;



	

//	/*clark变换*/
//	ClarkConvert();
//	/*park变换*/
//	ParkConvert();
//	/*更新theta值*/
//	RotorDetect();
//	/*电流环PI计算*/
//	m_pid.iq.Ref_In						=	m_motor_ctrl.m_motion_ctrl.f_set_iq;
//	m_pid.id.Ref_In						=	m_motor_ctrl.m_motion_ctrl.f_set_id;
//	m_motor_rt_para.m_revpark.f_v_q		=	Increment_PI_Cal(&(m_pid.iq),m_motor_rt_para.m_park.f_i_q);
//	m_motor_rt_para.m_revpark.f_v_d		=	Increment_PI_Cal(&(m_pid.id),m_motor_rt_para.m_park.f_i_d);
//	/*park逆变换*/
//	ReverseParkConvert();
//	/*根据V_alpha/beta计算参考电压落在哪个区间，并进行脉宽计算*/
//	CheckSector();
//	
//	/*对PID输出做出赋值限制*/
//	if(m_motor_rt_para.m_reverse.i16_T_1 > MAX_DUTY_CYCLE_INT16)
//		m_motor_rt_para.m_reverse.i16_T_1	=	MAX_DUTY_CYCLE_INT16;
//	if(m_motor_rt_para.m_reverse.i16_T_2 > MAX_DUTY_CYCLE_INT16)
//		m_motor_rt_para.m_reverse.i16_T_2	=	MAX_DUTY_CYCLE_INT16;
//	if(m_motor_rt_para.m_reverse.i16_T_3 > MAX_DUTY_CYCLE_INT16)
//		m_motor_rt_para.m_reverse.i16_T_3	=	MAX_DUTY_CYCLE_INT16;
//	if(m_motor_rt_para.m_reverse.i16_T_1 < MIN_DUTY_CYCLE_INT16)
//		m_motor_rt_para.m_reverse.i16_T_1	=	MIN_DUTY_CYCLE_INT16;
//	if(m_motor_rt_para.m_reverse.i16_T_2 < MIN_DUTY_CYCLE_INT16)
//		m_motor_rt_para.m_reverse.i16_T_2	=	MIN_DUTY_CYCLE_INT16;
//	if(m_motor_rt_para.m_reverse.i16_T_3 < MIN_DUTY_CYCLE_INT16)
//		m_motor_rt_para.m_reverse.i16_T_3	=	MIN_DUTY_CYCLE_INT16;
//	
//	/*更改PWM占空比*/
//	TIM1->CCR1				=	4200 - (uint16_t)m_motor_rt_para.m_reverse.i16_T_1;
//	TIM1->CCR2				=	4200 - (uint16_t)m_motor_rt_para.m_reverse.i16_T_2;
//	TIM1->CCR3				=	4200 - (uint16_t)m_motor_rt_para.m_reverse.i16_T_3;
//	
}


void	(*runtime_svpwm_switch_table[2][8])()	=	{	{	U_OFF_V_OFF_W_OFF,
															U_OFF_V_OFF_W_ON,
															U_OFF_V_ON_W_OFF,
															U_OFF_V_ON_W_ON,
															U_ON_V_OFF_W_OFF,
															U_ON_V_OFF_W_ON,
															U_ON_V_ON_W_OFF,
															U_ON_V_ON_W_ON},
														{	U_ON_V_ON_W_ON,
															U_ON_V_ON_W_OFF,
															U_ON_V_OFF_W_ON,
															U_ON_V_OFF_W_OFF,
															U_OFF_V_ON_W_ON,
															U_OFF_V_ON_W_OFF,
															U_OFF_V_OFF_W_ON,
															U_OFF_V_OFF_W_OFF}}; 







/**************************************END OF FILE*****************************************/

