/**
  ******************************************************************************
  * @file    Project/user/hallreversal.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170928
  * @brief   ���������Լ�SVPWM����
  ******************************************************************************
  ******************************************************************************
  */
#include	"stm32f4xx.h"
#include	"stm32f4xx_gpio.h"
#include	"stm32f4xx_tim.h"
#include	"global_parameters.h"
#include	"hall_reversal_svpwm.h"

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
	��������			��(void)
	��������			��null
	��������			��H�ſ��ع�״̬0�����ű۶��ر�
	----------------------------------------------------------------------------*/



	/*---------------------------------------------------------------------------
	��������			��(void)
	��������			��null
	��������			��H�ſ��ع�״̬0�����ű۶��ر�
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

