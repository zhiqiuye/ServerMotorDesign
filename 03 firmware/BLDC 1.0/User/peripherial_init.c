/**
  ******************************************************************************
  * @file    Project/ucosii stm32f4 mdk475/Peripherial_Init.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20170508
  * @brief   Peripherial init,includes LED,	SPI<->DMS, USART, and so on.
  ******************************************************************************
  ******************************************************************************
  */


#include	"peripherial_init.h"
#include	"stm32f4xx_usart.h"
#include	"stm32f4xx_rcc.h"
#include	"stm32f4xx_gpio.h"
#include	"stm32f4xx_spi.h"
#include	"stm32f4xx_tim.h"
#include	"stm32f4xx_dma.h"
#include	"stm32f4xx_adc.h"
#include	"stm32f4xx_can.h"
#include	"ucos_ii.h"
#include	"includes.h"
#include	"global_parameters.h"
#include	"delay.h"

//uint16_t	curr_temp[12];


	/*---------------------------------------------------------------------------
	��������			��NVIC_Config(void)
	��������			��null
	��������			��
	----------------------------------------------------------------------------*/
void	NVIC_Config(void)
{
	NVIC_InitTypeDef 	NVIC_InitStructure;
	
	/*TIM1 ��������ж�*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/*Hall ���������벶���ж�*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/*ADC-DMA ��������ж�*/
	NVIC_InitStructure.NVIC_IRQChannel						=	DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/**/
}
	
	/*---------------------------------------------------------------------------
	��������			��GPIO_Config(void)
	��������			��null
	��������			��STM32F405оƬ���ж������Ž��г�ʼ������
							
	----------------------------------------------------------------------------*/
void	GPIO_Config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
/*�����ܽ�ʱ��*/	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC,ENABLE);

/*LED�ܽ����� PB9*/	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_9;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_9);
	
/*�����ű�PWM��������*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_DOWN;									//��Ҫ��Ϊ�����������޷�����Ϊ�͵�ƽ
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM1);
	GPIO_ResetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	
/*�����������ӿ�*/

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);
	
/*�����������ӿ�*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);
	
/*ADģ�����ɼ����ݶ�ʹ��PA6/7�Լ�PC4/5,20180622*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
	
/*��ű�բ���ƶ� PC13����ű�բ�����ź�PC14 -> close��PC15 -> open*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_13;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_2MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_2MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_14|GPIO_Pin_15);
	
}


	/*---------------------------------------------------------------------------
	��������			��USART2_DMA_Config(void)
	��������			��null
	��������			��RS485��
	----------------------------------------------------------------------------*/
void	USART2_DMA_Config(void)
{

}


	/*---------------------------------------------------------------------------
	��������			��USART3_DMA_Config(void)
	��������			��null
	��������			��USB-������
	----------------------------------------------------------------------------*/
void	USART3_DMA_Config(void)
{

}


	/*---------------------------------------------------------------------------
	��������			��SPI1_DMA_Config(void)
	��������			��null
	��������			��������оƬW5500ͨѶʹ��
	----------------------------------------------------------------------------*/
void	SPI1_DMA_Config(void)
{

}


	/*---------------------------------------------------------------------------
	��������			��Timer1_Config(void)
	��������			��null
	��������			��	��·H�ŵ������ź�				timer1 APB2�� 84MHz
							PWMʹ��20KHz
							PWMռ�ձȵ��ڷ�Χ 0-4200
							������ʽ�����������¼���
							�ڼ������ݼ�ʱ�����Ƚ��ж�
							��Ч��ƽΪ��
	----------------------------------------------------------------------------*/
void	Timer1_Config(void)
{
	TIM_TimeBaseInitTypeDef	TIM_BaseInitStructure;
	TIM_OCInitTypeDef		TIM_OCInitStructure;
	TIM_BDTRInitTypeDef		TIM_BDTRInitStructure;

/*ʱ��ʹ��*/	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
/*��ʼ����ʱ���Ļ�������*/
	TIM_DeInit(TIM1);
	TIM_BaseInitStructure.TIM_ClockDivision			=	TIM_CKD_DIV1;							//ʱ�ӷָ�Ϊ1����ʹ��84MHzʱ��
	TIM_BaseInitStructure.TIM_CounterMode			=	TIM_CounterMode_CenterAligned1;			//���Ķ���ģʽ1���ڼ������ݼ�ʱ�����Ƚ��ж�
																								//���Ķ���ģʽ2���ڼ���������ʱ�����Ƚ��ж�
																								//���Ķ���ģʽ3���ڼ�����������ݼ�ʱ�����Ƚ��ж�
	TIM_BaseInitStructure.TIM_Period				=	4200-1;									//����ֵ�������
	TIM_BaseInitStructure.TIM_Prescaler				=	1;										//Ԥ��Ƶ��Ϊ1
	TIM_BaseInitStructure.TIM_RepetitionCounter		=	0;
	TIM_TimeBaseInit(TIM1,&TIM_BaseInitStructure);
	
/*��ʼ����ʱ���ĸ�ͨ��*/
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode					=	TIM_OCMode_PWM2;						//pwmģʽ1��������ϼ���ʱ��cnt<ccrʱΪ��Ч��ƽ�����¼���ʱ��cnt>ccrʱΪ��Ч��ƽ
																								//pwmģʽ2��������ϼ���ʱ��cnt<ccrʱΪ��Ч��ƽ�����¼���ʱ��cnt>ccrʱΪ��Ч��ƽ
	TIM_OCInitStructure.TIM_OCIdleState				=	TIM_OCIdleState_Reset;					//������pulseֵ�����͹ܽ�
	TIM_OCInitStructure.TIM_OCNIdleState			=	TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCPolarity				=	TIM_OCPolarity_Low;						//��Ч��ƽΪ��
	TIM_OCInitStructure.TIM_OCNPolarity				=	TIM_OCNPolarity_Low;					//�������ͨ������Ч��ƽΪ�ͣ���TIM_OCPolarity����һ�²��ܻ���
	TIM_OCInitStructure.TIM_OutputState				=	TIM_OutputState_Enable;					//���ͨ��ʹ��
	TIM_OCInitStructure.TIM_OutputNState			=	TIM_OutputNState_Enable;				//�������ͨ�������ʹ��
	TIM_OCInitStructure.TIM_Pulse					=	10;									//ռ�ձ�����
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_Pulse					=	10;
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_Pulse					=	10;
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);

/*��������������жϣ����ڲ����������¼�����������������Periodֵʱ������PWM�ߵ�ƽ���е�λ�ã���ʱ�жϽ��е�������*/
	TIM_ClearFlag(TIM1,TIM_IT_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);

/*��������*/
	TIM_BDTRInitStructure.TIM_OSSRState				=	TIM_OSSRState_Disable;					//����ģʽ�����ѡ��
	TIM_BDTRInitStructure.TIM_OSSIState				=	TIM_OSSIState_Enable;					//����ģʽ�����ѡ��
	TIM_BDTRInitStructure.TIM_LOCKLevel				=	TIM_LOCKLevel_OFF;						//��������
	TIM_BDTRInitStructure.TIM_DeadTime				=	0x64;									//����ʱ��
	TIM_BDTRInitStructure.TIM_Break					=	TIM_Break_Disable;						//ɲ������ʹ��
	TIM_BDTRInitStructure.TIM_BreakPolarity			=	TIM_BreakPolarity_High;					//ɲ������
	TIM_BDTRInitStructure.TIM_AutomaticOutput		=	TIM_AutomaticOutput_Enable;				//�Զ����ʹ��
	TIM_BDTRConfig(TIM1,&TIM_BDTRInitStructure);

	TIM_ARRPreloadConfig(TIM1,ENABLE);
	TIM_Cmd(TIM1,DISABLE);
	TIM_CtrlPWMOutputs(TIM1,DISABLE);
}

//CH1	��Ч��ƽΪ��--------------------------------------------------------------------------------------------------
	/*---------------------------------------------------------------------------
	��������			��TIM1_CH1_OFF_CH1N_ON(void)
	��������			��null
	��������			�����ű۹�		���űۿ�
	----------------------------------------------------------------------------*/
void	TIM1_CH1_OFF_CH1N_ON(void)
{
	TIM1->CCER &=0xFFFE;			//CCER->CC1E	= 0
	TIM1->CCER |=0x0004;			//CCER->CC1NE	= 1
	TIM1->CCMR1 &= 0xFF8F;
	TIM1->CCMR1 |= 0x0040;			//ǿ��OC1�����Ч��ƽ
}

	/*---------------------------------------------------------------------------
	��������			��TIM1_CH1_OFF_CH1N_OFF(void)
	��������			��null
	��������			�����ű۹�		���ű۹�
	----------------------------------------------------------------------------*/
void	TIM1_CH1_OFF_CH1N_OFF(void)
{
	TIM1->CCER &=0xFFFE;			//CCER->CC1E	= 0
	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	= 0
	TIM1->CCMR1 &= 0xFF8F;
	TIM1->CCMR1 |= 0x0040;			//ǿ��OC1�����Ч��ƽ
}

	/*---------------------------------------------------------------------------
	��������			��TIM1_CH1_ON_CH1N_OFF(void)
	��������			��null
	��������			�����űۿ�		���ű۹�
	----------------------------------------------------------------------------*/
void	TIM1_CH1_ON_CH1N_OFF(void)
{
	TIM1->CCER |=0x0001;			//CCER->CC1E	= 1
	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	= 0
	TIM1->CCMR1 &= 0xFF8F;
	TIM1->CCMR1 |= 0x0040;			//ǿ��OC1�����Ч��ƽ
}

	/*---------------------------------------------------------------------------
	��������			��TIM1_CH1_OFF_CH1N_PWM(void)
	��������			��null
	��������			�����ű۹�		���ű�pwm
	----------------------------------------------------------------------------*/
void	TIM1_CH1_OFF_CH1N_PWM(void)
{
	TIM1->CCMR1 &= 0xFF8F;
	TIM1->CCMR1 |= 0x0070;			//OC1ΪPWM2ģʽ
	TIM1->CCER &=0xFFFE;			//CCER->CC1E	=	0
	TIM1->CCER |=0x0004;			//CCER->CC1NE	=	1
}


	/*---------------------------------------------------------------------------
	��������			��TIM1_CH1_PWM_CH1N_OFF(void)
	��������			��null
	��������			�����ű�pwm		���ű۹�
	----------------------------------------------------------------------------*/
void	TIM1_CH1_PWM_CH1N_OFF(void)
{
	TIM1->CCMR1 &= 0xFF8F;
	TIM1->CCMR1 |= 0x0070;			//OC1ΪPWM2ģʽ
	TIM1->CCER |=0x0001;			//CCER->CC1E	=	1
	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	=	0
}

//CH2------------------------------------------------------------------------------------------------------------------
	/*---------------------------------------------------------------------------
	��������			��TIM1_CH2_OFF_CH2N_ON(void)
	��������			��null
	��������			�����ű۹�		���űۿ�
	----------------------------------------------------------------------------*/
void	TIM1_CH2_OFF_CH2N_ON(void)
{
	TIM1->CCER &=0xFFEF;			//CCER->CC2E	= 0
	TIM1->CCER |=0x0040;			//CCER->CC2NE	= 1
	TIM1->CCMR1 &= 0x8FFF;
	TIM1->CCMR1 |= 0x4000;			//ǿ��OC1�����Ч��ƽ
}

	/*---------------------------------------------------------------------------
	��������			��TIM1_CH2_OFF_CH2N_OFF(void)
	��������			��null
	��������			�����ű۹�		���ű۹�
	----------------------------------------------------------------------------*/
void	TIM1_CH2_OFF_CH2N_OFF(void)
{
	TIM1->CCER &=0xFFEF;			//CCER->CC2E	= 0
	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	= 0
	TIM1->CCMR1 &= 0x8FFF;
	TIM1->CCMR1 |= 0x4000;			//ǿ��OC1�����Ч��ƽ
}

	/*---------------------------------------------------------------------------
	��������			��TIM1_CH2_ON_CH2N_OFF(void)
	��������			��null
	��������			�����űۿ�		���ű۹�
	----------------------------------------------------------------------------*/
void	TIM1_CH2_ON_CH2N_OFF(void)
{
	TIM1->CCER |=0x0010;			//CCER->CC2E	= 1
	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	= 0
	TIM1->CCMR1 &= 0x8FFF;
	TIM1->CCMR1 |= 0x4000;			//ǿ��OC2�����Ч��ƽ
}

	/*---------------------------------------------------------------------------
	��������			��TIM1_CH2_OFF_CH2N_PWM(void)
	��������			��null
	��������			�����ű۹�		���ű�pwm
	----------------------------------------------------------------------------*/
void	TIM1_CH2_OFF_CH2N_PWM(void)
{
	TIM1->CCMR1 &= 0x8FFF;
	TIM1->CCMR1 |= 0x7000;			//OC2ΪPWM2ģʽ
	TIM1->CCER &=0xFFEF;			//CCER->CC2E	=	0
	TIM1->CCER |=0x0040;			//CCER->CC2NE	=	1
}


	/*---------------------------------------------------------------------------
	��������			��TIM1_CH2_PWM_CH2N_OFF(void)
	��������			��null
	��������			�����ű�pwm		���ű۹�
	----------------------------------------------------------------------------*/
void	TIM1_CH2_PWM_CH2N_OFF(void)
{
	TIM1->CCMR1 &= 0x8FFF;
	TIM1->CCMR1 |= 0x7000;			//OC2ΪPWM2ģʽ
	TIM1->CCER |=0x0010;			//CCER->CC2E	=	1
	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	=	0
}
//CH3-------------------------------------------------------------------------------------------------------------------------
	/*---------------------------------------------------------------------------
	��������			��TIM1_CH3_OFF_CH3N_ON(void)
	��������			��null
	��������			�����ű۹�		���űۿ�
	----------------------------------------------------------------------------*/
void	TIM1_CH3_OFF_CH3N_ON(void)
{
	TIM1->CCER &=0xFEFF;			//CCER->CC3E	= 0
	TIM1->CCER |=0x0400;			//CCER->CC3NE	= 1
	TIM1->CCMR2 &= 0xFF8F;
	TIM1->CCMR2 |= 0x0040;			//ǿ��OC3�����Ч��ƽ
}

	/*---------------------------------------------------------------------------
	��������			��TIM1_CH3_OFF_CH3N_OFF(void)
	��������			��null
	��������			�����ű۹�		���ű۹�
	----------------------------------------------------------------------------*/
void	TIM1_CH3_OFF_CH3N_OFF(void)
{
	TIM1->CCER &=0xFEFF;			//CCER->CC3E	= 0
	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	= 0
	TIM1->CCMR2 &= 0xFF8F;
	TIM1->CCMR2 |= 0x0040;			//ǿ��OC3�����Ч��ƽ
}

	/*---------------------------------------------------------------------------
	��������			��TIM1_CH3_ON_CH3N_OFF(void)
	��������			��null
	��������			�����űۿ�		���ű۹�
	----------------------------------------------------------------------------*/
void	TIM1_CH3_ON_CH3N_OFF(void)
{
	TIM1->CCER |=0x0100;			//CCER->CC3E	= 1
	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	= 0
	TIM1->CCMR2 &= 0xFF8F;
	TIM1->CCMR2 |= 0x0040;			//ǿ��OC3�����Ч��ƽ
}

	/*---------------------------------------------------------------------------
	��������			��TIM1_CH3_OFF_CH3N_PWM(void)
	��������			��null
	��������			�����ű۹�		���ű�pwm
	----------------------------------------------------------------------------*/
void	TIM1_CH3_OFF_CH3N_PWM(void)
{
	TIM1->CCMR2 &= 0xFF8F;
	TIM1->CCMR2 |= 0x0070;			//OC3ΪPWM2ģʽ
	TIM1->CCER &=0xFEFF;			//CCER->CC3E	=	0
	TIM1->CCER |=0x0400;			//CCER->CC3NE	=	1
}


	/*---------------------------------------------------------------------------
	��������			��TIM1_CH3_PWM_CH3N_OFF(void)
	��������			��null
	��������			�����ű�pwm		���ű۹�
	----------------------------------------------------------------------------*/
void	TIM1_CH3_PWM_CH3N_OFF(void)
{
	TIM1->CCMR2 &= 0xFF8F;
	TIM1->CCMR2 |= 0x0070;			//OC3ΪPWM2ģʽ
	TIM1->CCER |=0x0100;			//CCER->CC3E	=	1
	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	=	0
}


	/*---------------------------------------------------------------------------
	��������			��Timer3_Config(void)
	��������			��null
	��������			�����ڽ��չ����������ź�			timer3 APB1��42MHz
	----------------------------------------------------------------------------*/
void	Timer3_Config(void)
{
	TIM_TimeBaseInitTypeDef 		TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef				TIM_ICInitStructure;	
	
/*ʱ��ʹ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
/*Timer3 ����*/
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_CounterMode			=	TIM_CounterMode_Up;					//���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision			=	TIM_CKD_DIV1;						//TIM3ʱ�ӷָʹ��42MHzƵ��
	TIM_TimeBaseInitStructure.TIM_Period				=	10;								//�����嵽���ֵʱ������װ�أ�������һ���ж�
	TIM_TimeBaseInitStructure.TIM_Prescaler				=	0;									//
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
/*���ö�ʱ��Ϊ������ģʽ��IT1,IT2Ϊ�����ؼ���*/
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel						=	TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter					=	1;									//�����˲���������1��ʱ�Ӽ�����������Ч
	TIM_ICInit(TIM3,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel						=	TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter					=	1;									//�����˲���������1��ʱ�Ӽ�����������Ч
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM3->CNT = 0;
	TIM_Cmd(TIM3,ENABLE);	
}


	/*---------------------------------------------------------------------------
	��������			��Timer4_Config(void)
	��������			��null
	��������			��	���ڼ�����������״̬			timer4 APB1��42MHz
							16bit��ʱ��
							TIM4��CH1/CH2/CH3����Ϊ���벶����CH1��������ͨ�����
							������ͨ���ź��б�ʱ���ᴥ��TIM4���жϣ����жϺ����н���
							���������
	----------------------------------------------------------------------------*/
void	Timer4_Config(void)
{
	TIM_TimeBaseInitTypeDef	TIM_BaseInitStructure;
	TIM_ICInitTypeDef		TIM_ICInitStructure;

/*ʱ�ӿ���*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
/*��ʱ����������*/
	TIM_BaseInitStructure.TIM_ClockDivision			=	0;									//ʱ�ӷ�Ƶϵ��1��ʹ��42Mhzʱ��Ƶ��
	TIM_BaseInitStructure.TIM_CounterMode			=	TIM_CounterMode_Up;					//���ϼ���ģʽ
	TIM_BaseInitStructure.TIM_Period				=	65535;								//�������ֵ
	TIM_BaseInitStructure.TIM_Prescaler				=	0;									//Ԥ��Ƶϵ��1��ʹ��42MHz
	TIM_TimeBaseInit(TIM4,&TIM_BaseInitStructure);

/*ͨ����������*/	
	TIM_ICInitStructure.TIM_Channel					=	TIM_Channel_1;						//ͨ��һ
	TIM_ICInitStructure.TIM_ICPolarity				=	TIM_ICPolarity_BothEdge;			//�������������½���
	TIM_ICInitStructure.TIM_ICFilter				=	0;									//�������˲�
	TIM_ICInitStructure.TIM_ICPrescaler				=	0;									//�����źŲ���Ƶ
	TIM_ICInitStructure.TIM_ICSelection				=	TIM_ICSelection_DirectTI;			//ͨ����TIxֱ��
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel					=	TIM_Channel_2;
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel					=	TIM_Channel_3;
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
/*���û���������ģʽ����IC1/IC2/IC3����TI��������*/	
	TIM_SelectHallSensor(TIM4,ENABLE);
	
/*���������ж�*/
//	TIM_ClearFlag(TIM4,TIM_IT_CC1);
//	TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);													//ʹ�ܸ����ж�
	TIM_Cmd(TIM4,DISABLE);																	//������ʱ��4
}


	/*---------------------------------------------------------------------------
	��������			��ADC_DMA_Config(void)
	��������			��null
	��������			��ADC������DMA��ʼ��
							PA6				ADC12_IN_6
							PA7				ADC12_IN_7
							PC4				ADC12_IN_14
							PC5				ADC12_IN_15
							ADC2	DMA2  stream 0 channel 0
	----------------------------------------------------------------------------*/
void	ADC_DMA_Config(void)
{
	ADC_InitTypeDef			ADC_InitStructure;
	ADC_CommonInitTypeDef	ADC_CommonInitStructure;
	DMA_InitTypeDef			DMA_InitStructure;

	/*DMA ���ã�ʹ��DMA2_Channel_0 Stream_0*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel					=	DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr		=	(uint32_t)(&ADC1->DR);					//ʹ��ADC3�����ݼĴ�����ΪDMA�������ַ
	DMA_InitStructure.DMA_Memory0BaseAddr			=	(uint32_t)(&(m_motor_rt_para.ADC_DMA_buf));//�������׵�ַ��Ϊ�ڴ��ַ
	DMA_InitStructure.DMA_DIR						=	DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize				=	32;
	DMA_InitStructure.DMA_PeripheralInc				=	DMA_PeripheralInc_Disable;				//DMA��������������ַ������
	DMA_InitStructure.DMA_MemoryInc					=	DMA_MemoryInc_Enable;					//DMA����������ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize		=	DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize			=	DMA_MemoryDataSize_HalfWord;			//ʹ��16λ������Ϊ���䵥λ
	DMA_InitStructure.DMA_Mode						=	DMA_Mode_Circular;						//����DMA����ģʽΪѭ������
	DMA_InitStructure.DMA_Priority					=	DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode					=	DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold				=	DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst				=	DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst			=	DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0,ENABLE);																//ʹ��ADC��DMA

/*ADC1 ����*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);										//ADC������APB2��
	
	ADC_DeInit();
	
	ADC_CommonInitStructure.ADC_DMAAccessMode		=	ADC_DMAAccessMode_Disabled;				//
	ADC_CommonInitStructure.ADC_Mode				=	ADC_Mode_Independent;					//ֻʹ��һ·ADC������ͬ���������ö���ģʽ����
	/*ADC��ʱ����Ϊ 84MHz/8 =10.5MHz*/
	ADC_CommonInitStructure.ADC_Prescaler			=	ADC_Prescaler_Div4;						//APB2����ʱ��Ƶ��Ϊ84MHz�������÷�Ƶϵ��2/4/6/8��ADC��ʱ����Ҫ������14MHz���²ſ�������ʹ��??
	/*����������Ϊ20KHz����ѹֵ������Ҫ����320KHz��ÿ��32��ʱ�Ӳ�һ����*/
	ADC_CommonInitStructure.ADC_TwoSamplingDelay	=	ADC_TwoSamplingDelay_5Cycles;			//�������β���ʱ�Ӽ����������Ϊ5-20��ÿͨ��ת��ʱ��=����ʱ��+12��ADCʱ������=15��ʱ������
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	ADC_InitStructure.ADC_Resolution				=	ADC_Resolution_12b;						//�����ֱ���Ϊ12λ
	ADC_InitStructure.ADC_ScanConvMode				=	ENABLE;									//ɨ��ģʽ(��ͨ��ADC�ɼ�Ҫ��ɨ��ģʽ)
	ADC_InitStructure.ADC_ContinuousConvMode		=	ENABLE;									//�ر�����ת��
	ADC_InitStructure.ADC_ExternalTrigConv			=	ADC_ExternalTrigConvEdge_None;			//û���ⲿ����
	ADC_InitStructure.ADC_DataAlign					=	ADC_DataAlign_Right;					//���ݸ�ʽ�Ҷ���
	ADC_InitStructure.ADC_NbrOfConversion			=	4;										//��ʹ��6��ͨ��
	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_6,1,ADC_SampleTime_15Cycles);						//����ģʽͨ�����ã�������������ѹ
	ADC_RegularChannelConfig(ADC1,ADC_Channel_7,2,ADC_SampleTime_15Cycles);						//����ģʽͨ�����ã�ĸ�ߵ�ѹ
	ADC_RegularChannelConfig(ADC1,ADC_Channel_14,3,ADC_SampleTime_15Cycles);					//����ģʽͨ�����ã�ѹ��������1��ѹ
	ADC_RegularChannelConfig(ADC1,ADC_Channel_15,4,ADC_SampleTime_15Cycles);					//����ģʽͨ�����ã�ѹ��������2��ѹ

/*DMA�ж�����*/
	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TC);
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);												//����DMA��������ж�
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);											//ʹ��DMA������ɺ�����ADC
	ADC_DMACmd(ADC1,ENABLE);																	//ʹ��ADC��DMA	
	ADC_Cmd(ADC1,ENABLE);																		//ʹ��ADC1
	ADC_SoftwareStartConv(ADC1);																//����ADC1�����ת��
}


	/*---------------------------------------------------------------------------
	��������			��CAN_Config(void)
	��������			��null
	��������			��
	----------------------------------------------------------------------------*/
void	CAN_Config(void)
{

}


	/*---------------------------------------------------------------------------
	��������			��BrakeControl(void)
	��������			��mode:		1 ������բ		0 �رձ�բ		���� �޲���
	��������			�������رձ�բʹ�ú���
	----------------------------------------------------------------------------*/
void	BrakeControl(uint8_t	mode)
{
	uint8_t 	cnt = 0;
	uint32_t	io_state;
	if(mode == 0)
	{
		GPIOC->ODR |= 0x2000;											//PC13�ø�
		//�̵����ϵ�� PC14 -> 0	PC15 -> 1 ָʾ״̬
//		while(1)
//		{
//			io_state	=	(GPIOC->IDR >> 14) & 0xFFFC;
//			if(io_state == 0x02)
//				break;
//			else cnt++;
//			delay_us(2000);
//			if(cnt >= 250)												//����0.5sû��ȷ�ϱ�բ�򿪣���Ϊ��բ�򿪴���
//			{
//				m_error.brake_err = open_error;
//				return;
//			}
//		}
	}
	else if(mode == 1)
	{
		GPIOC->ODR &= 0xDFFF;											//PC13�õ�
		//�̵����ϵ�� PC14 -> 1	PC15 -> 0 ָʾ״̬
//		while(1)
//		{
//			io_state	=	(GPIOC->IDR >> 14) & 0xFFFC;
//			if(io_state == 0x01)
//				break;
//			else cnt++;
//			delay_us(2000);
//			if(cnt >= 250)												//����0.5sû��ȷ�ϱ�բ�򿪣���Ϊ��բ�򿪴���
//			{
//				m_error.brake_err = close_error;
//				return;
//			}
//		}
	}
	else
		return;
}

	/*---------------------------------------------------------------------------
	��������			��LED_ON(void)
	��������			��
	��������			��LED����
	----------------------------------------------------------------------------*/
void	LED_ON(void)
{
	GPIOB->ODR |= 0x0200;
}

	/*---------------------------------------------------------------------------
	��������			��LED_OFF(void)
	��������			��
	��������			��LEDϨ��
	----------------------------------------------------------------------------*/
void	LED_OFF(void)
{
	GPIOB->ODR &= 0xFDFF;
}

/**********************end of file************************/
