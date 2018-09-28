/**
  ******************************************************************************
  * @file    Project/user/hallreversal.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170928
  * @brief   霍尔换向以及SVPWM操作
  ******************************************************************************
  ******************************************************************************
  */
#include	"stm32f4xx.h"
#include	"stm32f4xx_gpio.h"
#include	"stm32f4xx_tim.h"
#include	"global_parameters.h"
#include	"hall_reversal_svpwm.h"

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
	函数名称			：(void)
	参数含义			：null
	函数功能			：H桥开关管状态0，上桥臂都关闭
	----------------------------------------------------------------------------*/



	/*---------------------------------------------------------------------------
	函数名称			：(void)
	参数含义			：null
	函数功能			：H桥开关管状态0，上桥臂都关闭
	----------------------------------------------------------------------------*/





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

