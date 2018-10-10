/**
  ******************************************************************************
  * @file    Project/user/foc_reversal_svpwm.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170928
  * @brief   ���������Լ�SVPWM����
  ******************************************************************************
  ******************************************************************************
  */

/*
  FOC ���ƿ�ͼ                                                                                                         
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
                                                                                                                                             
                                                                                                                                             
        Clark�任��
					I_alpha = Ia
                    I_beta  = (Ia + 2*Ib)/(3^0.5)

        Park�任��
                    I_q	    = I_beta * cos(theta) - I_alpha * sin(theta)
                    I_d     = I_alpha * cos(theta) + I_beta * sin(theta)

        Reverse Park�任��
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

//����8������ֻ��SVPWMʱ�Ļ�������ȷ����Щ���عܵ�ͨ����Щ���عܹر�
//ռ�ձ���Ҫ�ڵ�������
//////////////////////////////////////////////////////////////////////////////////
	/*---------------------------------------------------------------------------
	��������			��	U_OFF_V_OFF_W_OFF(void)
	��������			��	null
	��������			��	H�ſ��ع�״̬			000��
						���ű�a/b/c���ر�
						���ű�a/b/c������
	----------------------------------------------------------------------------*/
void	U_OFF_V_OFF_W_OFF(void)
{
//	ch1 off-pwm
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1ΪPWM2ģʽ
//	TIM1->CCER &=0xFFFE;			//CCER->CC1E	=	0
//	TIM1->CCER |=0x0004;			//CCER->CC1NE	=	1
//	ch2 off-pwm
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2ΪPWM2ģʽ
//	TIM1->CCER &=0xFFEF;			//CCER->CC2E	=	0
//	TIM1->CCER |=0x0040;			//CCER->CC2NE	=	1
//	ch3 off-pwm
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3ΪPWM2ģʽ
//	TIM1->CCER &=0xFEFF;			//CCER->CC3E	=	0
//	TIM1->CCER |=0x0400;			//CCER->CC3NE	=	1
	
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2ΪPWM2ģʽ
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3ΪPWM2ģʽ
	TIM1->CCER	&=	0xFEEE;			//CCER->CC1/2/3E	=	0
	TIM1->CCER	|=	0x0444;			//CCER->CC1/2/3NE	=	1
}


	/*---------------------------------------------------------------------------
	��������			��	U_ON_V_OFF_W_OFF(void)
	��������			��	null
	��������			��	H�ſ��ع�״̬			100��
						���ű�a������b/c���ر�
						���ű�a�رգ�b/c������
	----------------------------------------------------------------------------*/
void	U_ON_V_OFF_W_OFF(void)
{
//	//ch1 pwm-off
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1ΪPWM2ģʽ
//	TIM1->CCER |=0x0001;			//CCER->CC1E	=	1
//	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	=	0
//	//ch2 off-pwm
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2ΪPWM2ģʽ
//	TIM1->CCER &=0xFFEF;			//CCER->CC2E	=	0
//	TIM1->CCER |=0x0040;			//CCER->CC2NE	=	1
//	//ch3 off-pwm
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3ΪPWM2ģʽ
//	TIM1->CCER &=0xFEFF;			//CCER->CC3E	=	0
//	TIM1->CCER |=0x0400;			//CCER->CC3NE	=	1
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2ΪPWM2ģʽ
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3ΪPWM2ģʽ
	TIM1->CCER	&=	0xFEEB;
	TIM1->CCER	|=	0x0441;
}


	/*---------------------------------------------------------------------------
	��������			��	U_OFF_V_ON_W_OFF(void)
	��������			��	null
	��������			��	H�ſ��ع�״̬			010��
						���ű�b������a/c���ر�
						���ű�b�رգ�a/c������
	----------------------------------------------------------------------------*/
void	U_OFF_V_ON_W_OFF(void)
{
//	//ch1 off-pwm
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1ΪPWM2ģʽ
//	TIM1->CCER &=0xFFFE;			//CCER->CC1E	=	0
//	TIM1->CCER |=0x0004;			//CCER->CC1NE	=	1
//	//ch2 pwm-off
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2ΪPWM2ģʽ
//	TIM1->CCER |=0x0010;			//CCER->CC2E	=	1
//	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	=	0
//	//ch3 off-pwm
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3ΪPWM2ģʽ
//	TIM1->CCER &=0xFEFF;			//CCER->CC3E	=	0
//	TIM1->CCER |=0x0400;			//CCER->CC3NE	=	1
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2ΪPWM2ģʽ
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3ΪPWM2ģʽ
	TIM1->CCER	&=	0xFEBE;			//CCER->CC1/3E	=	0		CCER->CC2E	=	1
	TIM1->CCER	|=	0x0414;			//CCER->CC1/3NE	=	1		CCER->CC2NE	=	0
}


	/*---------------------------------------------------------------------------
	��������			��	U_ON_V_ON_W_OFF(void)
	��������			��	null
	��������			��	H�ſ��ع�״̬			110��
						���ű�a/b������c�ر�
						���ű�a/b�رգ�c����
	----------------------------------------------------------------------------*/
void	U_ON_V_ON_W_OFF(void)
{
	//ch1 pwm-off
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1ΪPWM2ģʽ
//	TIM1->CCER |=0x0001;			//CCER->CC1E	=	1
//	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	=	0
	//ch2 pwm-off
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2ΪPWM2ģʽ
//	TIM1->CCER |=0x0010;			//CCER->CC2E	=	1
//	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	=	0	
	//ch3 off-pwm
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3ΪPWM2ģʽ
//	TIM1->CCER &=0xFEFF;			//CCER->CC3E	=	0
//	TIM1->CCER |=0x0400;			//CCER->CC3NE	=	1
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2ΪPWM2ģʽ
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3ΪPWM2ģʽ	
	TIM1->CCER 	|=	0x0411;			//CCER->CC1/2E	=	1	CCER->CC3NE	=	1
	TIM1->CCER 	&=	0xFEBB;			//CCER->CC1/2E	=	1	CCER->CC3E	=	0
}


	/*---------------------------------------------------------------------------
	��������			��	U_OFF_V_OFF_W_ON(void)
	��������			��	null
	��������			��	H�ſ��ع�״̬			001��
						���ű�c������a/b�ر�
						���ű�c�رգ�a/b����
	----------------------------------------------------------------------------*/
void	U_OFF_V_OFF_W_ON(void)
{
//	//ch1 off-pwm
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1ΪPWM2ģʽ
//	TIM1->CCER &=0xFFFE;			//CCER->CC1E	=	0
//	TIM1->CCER |=0x0004;			//CCER->CC1NE	=	1
//	//ch2 off-pwm
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2ΪPWM2ģʽ
//	TIM1->CCER &=0xFFEF;			//CCER->CC2E	=	0
//	TIM1->CCER |=0x0040;			//CCER->CC2NE	=	1
//	//ch3 pwm-off
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3ΪPWM2ģʽ
//	TIM1->CCER |=0x0100;			//CCER->CC3E	=	1
//	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	=	0	
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2ΪPWM2ģʽ
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3ΪPWM2ģʽ	
	TIM1->CCER 	&=	0xFBEE;			//CCER->CC1/2E	=	0		CCER->CC3NE	=	0
	TIM1->CCER 	|=	0x0144;			//CCER->CC1/2NE	=	1		CCER->CC3NE	=	0
}


	/*---------------------------------------------------------------------------
	��������			��	U_ON_V_OFF_W_ON(void)
	��������			��	null
	��������			��	H�ſ��ع�״̬			101��
						���ű�a/c������b�ر�
						���ű�a/c�رգ�b����
	----------------------------------------------------------------------------*/
void	U_ON_V_OFF_W_ON(void)
{
//	//ch1 pwm-off
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1ΪPWM2ģʽ
//	TIM1->CCER |=0x0001;			//CCER->CC1E	=	1
//	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	=	0	
//	//ch2 off-pwm
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2ΪPWM2ģʽ
//	TIM1->CCER &=0xFFEF;			//CCER->CC2E	=	0
//	TIM1->CCER |=0x0040;			//CCER->CC2NE	=	1
//	//ch3 pwm-off
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3ΪPWM2ģʽ
//	TIM1->CCER |=0x0100;			//CCER->CC3E	=	1
//	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	=	0
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2ΪPWM2ģʽ
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3ΪPWM2ģʽ
	TIM1->CCER 	|=	0x0141;			//CCER->CC1/3E	=	1		CCER->CC2E	=	0
	TIM1->CCER	&=	0xFBEB;			//CCER->CC1/3NE	=	0		CCER->CC2NE	=	1
}


	/*---------------------------------------------------------------------------
	��������			��	U_OFF_V_ON_W_ON(void)
	��������			��	null
	��������			��	H�ſ��ع�״̬			011��
						���ű�b/c������a�ر�
						���ű�b/c�رգ�a����
	----------------------------------------------------------------------------*/
void	U_OFF_V_ON_W_ON(void)
{
//	//ch1 off-pwm
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1ΪPWM2ģʽ
//	TIM1->CCER &=0xFFFE;			//CCER->CC1E	=	0
//	TIM1->CCER |=0x0004;			//CCER->CC1NE	=	1
//	//ch2 pwm-off
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2ΪPWM2ģʽ
//	TIM1->CCER |=0x0010;			//CCER->CC2E	=	1
//	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	=	0
//	//ch3 pwm-off
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3ΪPWM2ģʽ
//	TIM1->CCER |=0x0100;			//CCER->CC3E	=	1
//	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	=	0
	
	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2ΪPWM2ģʽ
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3ΪPWM2ģʽ
	TIM1->CCER	&=	0xFBBE;			//CCER->CC2/3NE	=	0	CCER->CC1E	=	0
	TIM1->CCER 	|=	0x0114;			//CCER->CC2/3E	=	1	CCER->CC1NE	=	1
}


	/*---------------------------------------------------------------------------
	��������			��	U_ON_V_ON_W_ON(void)
	��������			��	null
	��������			��	H�ſ��ع�״̬			111��
						���ű�a/b/c������
						���ű�a/b/c���ر�
	----------------------------------------------------------------------------*/
void	U_ON_V_ON_W_ON(void)
{
//	//ch1 pwm-off
//	TIM1->CCMR1 &= 0xFF8F;
//	TIM1->CCMR1 |= 0x0070;			//OC1ΪPWM2ģʽ
//	TIM1->CCER |=0x0001;			//CCER->CC1E	=	1
//	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	=	0	
//	//ch2 pwm-off
//	TIM1->CCMR1 &= 0x8FFF;
//	TIM1->CCMR1 |= 0x7000;			//OC2ΪPWM2ģʽ
//	TIM1->CCER |=0x0010;			//CCER->CC2E	=	1
//	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	=	0	
//	//ch3 pwm-off
//	TIM1->CCMR2 &= 0xFF8F;
//	TIM1->CCMR2 |= 0x0070;			//OC3ΪPWM2ģʽ
//	TIM1->CCER |=0x0100;			//CCER->CC3E	=	1
//	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	=	0

	TIM1->CCMR1 &=	0x8F8F;
	TIM1->CCMR1 |=	0x7070;			//OC1/2ΪPWM2ģʽ
	TIM1->CCMR2 &=	0xFF8F;
	TIM1->CCMR2 |=	0x0070;			//OC3ΪPWM2ģʽ
	TIM1->CCER 	|=	0x0111;			//CCER->CC1/2/3E	=	1
	TIM1->CCER	&=	0xFBBB;			//CCER->CC1/2/3NE	=	0
}

//////////////////////////////////////////////////////////////////////////////////
	/*---------------------------------------------------------------------------
	��������			��	Increment_PI_Cal(PID_Struct* pid)
	��������			��	null
	��������			��	����PI������̣�
						���غ���
						���ֵ��ΪV_alpha��V_beta�����Ϊ3^0.5/3=0.57735Udc=13.856f
	----------------------------------------------------------------------------*/
float	Increment_PI_Cal(PID_struct* pid,float new_feedback)
{
	pid->Feed_Back			=	new_feedback;													//���·���ֵ
	pid->Err_T_1			=	pid->Err_T_0;													//����T-1ʱ�����
	pid->Err_T_0			=	pid->Ref_In - pid->Feed_Back;									//�����µ����
	
	pid->P_Out				=	pid->Kp * (pid->Err_T_0 - pid->Err_T_1);						//��������
	pid->I_Out				=	pid->Ki * pid->Err_T_0;											//���ֲ���
	
	pid->pid_increment		=	pid->P_Out + pid->I_Out;										//����PI����
	
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
	��������			��	ClarkConvert(void)
	��������			��	null
	��������			��	�ɵ������������ת��Ϊalpha/beta����ϵ�µĵ���ֵ
						Clark�任��
									I_alpha = Ia
									I_beta  = (Ia + 2*Ib)/(3^0.5)
	----------------------------------------------------------------------------*/
void	ClarkConvert(void)
{
	m_motor_rt_para.m_clark.f_i_alpha		=	m_motor_rt_para.m_current_sensor.f_adc_U_I;
	m_motor_rt_para.m_clark.f_i_beta		=	0.577350269f * m_motor_rt_para.m_current_sensor.f_adc_U_I + 1.15470053837f * m_motor_rt_para.m_current_sensor.f_adc_V_I;
}


	/*---------------------------------------------------------------------------
	��������			��	ParkConvert(void)
	��������			��	null
	��������			��	��alpha/beta����ϵ�µ���ת��Ϊd/q����ϵ��
						Park�任��
									I_q	    = I_beta * cos(theta) - I_alpha * sin(theta)
									I_d     = I_alpha * cos(theta) + I_beta * sin(theta)

						��Ҫ��ȡ���µ�ת�ӵ�Ƕ�
	----------------------------------------------------------------------------*/
void	ParkConvert(void)
{
	float	theta_rad;
	
	theta_rad							=	m_motor_rt_para.m_park.f_theta_rad + m_motor_attribute_para.m_motor_ind_att.f_delta_theta_0 ;					//ת�ӵ�Ƕ�ת��Ϊ���ȣ����ڼ���������
	m_motor_rt_para.m_park.f_sin_theta	=	arm_sin_f32(theta_rad);
	m_motor_rt_para.m_park.f_cos_theta	=	arm_cos_f32(theta_rad);
	
	m_motor_rt_para.m_park.f_i_q		=	m_motor_rt_para.m_clark.f_i_beta * m_motor_rt_para.m_park.f_cos_theta
											- m_motor_rt_para.m_clark.f_i_alpha * m_motor_rt_para.m_park.f_sin_theta;
	
	m_motor_rt_para.m_park.f_i_d		=	m_motor_rt_para.m_clark.f_i_alpha * m_motor_rt_para.m_park.f_cos_theta
											+ m_motor_rt_para.m_clark.f_i_beta * m_motor_rt_para.m_park.f_sin_theta;
}



	/*---------------------------------------------------------------------------
	��������			��	RotorDetect(void)
	��������			��	null
	��������			��	����ת��λ����Ϣ��thetaֵ
						�ø��ٶ˱��������ݸ���
						
						��Ҫ��Hall�����ж��н���
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
	��������			��	RotorCorrection(void)
	��������			��	null
	��������			��	����ת��λ����Ϣ��thetaֵ
						������������I�ಶ���ж��н��н�������

	----------------------------------------------------------------------------*/
void	RotorCorrection(void)
{
	m_motor_rt_para.m_park.f_theta_rad		=	m_motor_attribute_para.m_motor_ind_att.f_delta_theta_0;
	
	
	
//	if(m_motor_ctrl.m_motion_ctrl.u8_dir == 0)							//��ת
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
//	else																//��ת
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
	��������			��	RotorRecognition(uint8_t hall_state)
	��������			��	null
	��������			��	����ת��λ����Ϣ��thetaֵ
						������ʱת�ӵĽǶȹ��ƣ�����Hall״̬����
	----------------------------------------------------------------------------*/
void	RotorRecognition(uint8_t hall_state)
{
	if(m_motor_ctrl.m_motion_ctrl.u8_dir == 0)	//˳ʱ��ת
	{
		switch (hall_state)
		{
			case	4:													//0-60deg		��Ϊ��90deg
				m_motor_rt_para.m_park.f_theta_rad	=	1.5707963267948966192313216916398f;
				break;
			case	6:													//60-120deg		��Ϊ��150deg
				m_motor_rt_para.m_park.f_theta_rad	=	2.6179938779914943653855361527329f;
				break;
			case	2:													//120-180deg	��Ϊ��210deg
				m_motor_rt_para.m_park.f_theta_rad	=	3.6651914291880921115397506138261f;
				break;
			case	3:													//180-240deg	��Ϊ��270deg
				m_motor_rt_para.m_park.f_theta_rad	=	4.7123889803846898576939650749193f;
				break;
			case	1:													//240-300deg	��Ϊ��330deg
				m_motor_rt_para.m_park.f_theta_rad	=	5.7595865315812876038481795360124f;
				break;
			case	5:													//300-360deg	��Ϊ��30deg
				m_motor_rt_para.m_park.f_theta_rad	=	0.52359877559829887307710723054658f;
				break;
			default:
				//hall״̬����
				break;
		}	
	}
	else
	{
		
	}
}



	/*---------------------------------------------------------------------------
	��������			��	ReverseParkConvert(void)
	��������			��	null
	��������			��	��/q����ϵ�µ���ת��Ϊalpha/betad����ϵ��
						Reverse Park�任��
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
	��������			��	CheckSector(void)
	��������			��	null
	��������			��	����ο���ѹ��������
						�ο���ѹӦ�ó�ǰת��һ���Ƕȣ���˲����û���״̬��������λ�ж�
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
		case 3:				//������1
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
		case 1:				//������2
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
		
		case 5:				//������3
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
		case 4:				//������4
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

		case 6:				//������5
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
		
		case 2:				//������6
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
	��������			��	IsRotorStatic(void)
	��������			��	null
	��������			��	�жϵ��ת���Ƿ�ֹ
	----------------------------------------------------------------------------*/
void	IsRotorStatic(void)
{
	if(m_motor_ctrl.m_motion_ctrl.u8_speed_read_data_refreshed		==	1)						//������������ˢ��
	{
		if(m_motor_ctrl.m_motion_ctrl.u8_foc_state					==	foc_init_state_0)		//����FOC��ʼ��0�׶�
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
	��������			��	FOC_Cal(void)
	��������			��	null
	��������			��	FOC����
	----------------------------------------------------------------------------*/
void	FOC_Cal(void)
{
	/*����foc��ʼ��״̬������Ӧ����*/
	switch (m_motor_ctrl.m_motion_ctrl.u8_foc_state)
	{
		case foc_init_state_0:										/**< foc������Ƕ�Ϊ0���ȴ����ת���ȶ�*/
			m_motor_rt_para.m_revpark.f_v_q		=	10.5f;
			m_motor_rt_para.m_revpark.f_v_d		=	0.0f;
			m_motor_rt_para.m_park.f_cos_theta	=	arm_cos_f32(m_motor_rt_para.m_park.f_theta_rad);
			m_motor_rt_para.m_park.f_sin_theta	=	arm_sin_f32(m_motor_rt_para.m_park.f_theta_rad);
			IsRotorStatic();										//���ת���Ƿ�ֹ����ֹ�ͽ����¸�״̬
			break;
		
		case foc_init_state_1:										/**< ��һ�����ת���ȶ���ʹ��С��������ת�ӽ�����ת*/
			m_motor_rt_para.m_revpark.f_v_q		=	6.5f;
			m_motor_rt_para.m_revpark.f_v_d		=	0.0f;
			m_motor_rt_para.m_park.f_theta_rad	+= 0.0003141f;
			if(m_motor_rt_para.m_park.f_theta_rad >= 6.283185307179586476925286766559f)
				m_motor_rt_para.m_park.f_theta_rad = 0.0f;
			m_motor_rt_para.m_park.f_cos_theta	=	arm_cos_f32(m_motor_rt_para.m_park.f_theta_rad);
			m_motor_rt_para.m_park.f_sin_theta	=	arm_sin_f32(m_motor_rt_para.m_park.f_theta_rad);		//����TIM3_ch3�����жϺ�����һ������
			break;
		
		case foc_init_state_2:										/**< ת������ת��0��λ��*/
			m_motor_rt_para.m_revpark.f_v_q		=	6.5f;
			m_motor_rt_para.m_revpark.f_v_d		=	0.0f;
			m_motor_rt_para.m_park.f_cos_theta	=	arm_cos_f32(m_motor_rt_para.m_park.f_theta_rad);
			m_motor_rt_para.m_park.f_sin_theta	=	arm_sin_f32(m_motor_rt_para.m_park.f_theta_rad);
			IsRotorStatic();										//���ת���Ƿ�ֹ����ֹ�ͽ����¸�״̬
			break;
		
		case foc_init_over:											/**< FOC׼�����*/
			/*clark�任*/
			ClarkConvert();
			/*park�任*/
			ParkConvert();
			/*����thetaֵ*/
			RotorDetect();
			/*������PI����*/
			m_pid.iq.Ref_In						=	m_motor_ctrl.m_motion_ctrl.f_set_iq;
			m_pid.id.Ref_In						=	m_motor_ctrl.m_motion_ctrl.f_set_id;
			m_motor_rt_para.m_revpark.f_v_q		=	Increment_PI_Cal(&(m_pid.iq),m_motor_rt_para.m_park.f_i_q);
			m_motor_rt_para.m_revpark.f_v_d		=	Increment_PI_Cal(&(m_pid.id),m_motor_rt_para.m_park.f_i_d);
			break;
		
		default:
			break;
	}
	
	/*park��任*/
	ReverseParkConvert();
	CheckSector();
	
	/*��PID���������ֵ����*/
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
	
	/*����PWMռ�ձ�*/
	TIM1->CCR1				=	4200 - (uint16_t)m_motor_rt_para.m_reverse.i16_T_1;
	TIM1->CCR2				=	4200 - (uint16_t)m_motor_rt_para.m_reverse.i16_T_2;
	TIM1->CCR3				=	4200 - (uint16_t)m_motor_rt_para.m_reverse.i16_T_3;



	

//	/*clark�任*/
//	ClarkConvert();
//	/*park�任*/
//	ParkConvert();
//	/*����thetaֵ*/
//	RotorDetect();
//	/*������PI����*/
//	m_pid.iq.Ref_In						=	m_motor_ctrl.m_motion_ctrl.f_set_iq;
//	m_pid.id.Ref_In						=	m_motor_ctrl.m_motion_ctrl.f_set_id;
//	m_motor_rt_para.m_revpark.f_v_q		=	Increment_PI_Cal(&(m_pid.iq),m_motor_rt_para.m_park.f_i_q);
//	m_motor_rt_para.m_revpark.f_v_d		=	Increment_PI_Cal(&(m_pid.id),m_motor_rt_para.m_park.f_i_d);
//	/*park��任*/
//	ReverseParkConvert();
//	/*����V_alpha/beta����ο���ѹ�����ĸ����䣬�������������*/
//	CheckSector();
//	
//	/*��PID���������ֵ����*/
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
//	/*����PWMռ�ձ�*/
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

