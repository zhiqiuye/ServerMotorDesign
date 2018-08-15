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
	函数名称			：NVIC_Config(void)
	参数含义			：null
	函数功能			：
	----------------------------------------------------------------------------*/
void	NVIC_Config(void)
{
	NVIC_InitTypeDef 	NVIC_InitStructure;
	
	/*TIM1 计数溢出中断*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/*Hall 传感器输入捕获中断*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/*ADC-DMA 传输完成中断*/
	NVIC_InitStructure.NVIC_IRQChannel						=	DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/**/
}
	
	/*---------------------------------------------------------------------------
	函数名称			：GPIO_Config(void)
	参数含义			：null
	函数功能			：STM32F405芯片所有对外引脚进行初始化配置
							
	----------------------------------------------------------------------------*/
void	GPIO_Config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
/*开启管脚时钟*/	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC,ENABLE);

/*LED管脚配置 PB9*/	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_9;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_9);
	
/*上下桥臂PWM引脚配置*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_DOWN;									//需要改为下拉，否则无法驱动为低电平
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
	
/*增量编码器接口*/

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);
	
/*霍尔传感器接口*/
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
	
/*AD模拟量采集，暂定使用PA6/7以及PC4/5,20180622*/
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
	
/*电磁抱闸控制端 PC13，电磁抱闸输入信号PC14 -> close，PC15 -> open*/
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
	函数名称			：USART2_DMA_Config(void)
	参数含义			：null
	函数功能			：RS485用
	----------------------------------------------------------------------------*/
void	USART2_DMA_Config(void)
{

}


	/*---------------------------------------------------------------------------
	函数名称			：USART3_DMA_Config(void)
	参数含义			：null
	函数功能			：USB-串口用
	----------------------------------------------------------------------------*/
void	USART3_DMA_Config(void)
{

}


	/*---------------------------------------------------------------------------
	函数名称			：SPI1_DMA_Config(void)
	参数含义			：null
	函数功能			：与网卡芯片W5500通讯使用
	----------------------------------------------------------------------------*/
void	SPI1_DMA_Config(void)
{

}


	/*---------------------------------------------------------------------------
	函数名称			：Timer1_Config(void)
	参数含义			：null
	函数功能			：	三路H桥的驱动信号				timer1 APB2上 84MHz
							PWM使用20KHz
							PWM占空比调节范围 0-4200
							计数方式采用向上向下计数
							在计数器递减时产生比较中断
							有效电平为低
	----------------------------------------------------------------------------*/
void	Timer1_Config(void)
{
	TIM_TimeBaseInitTypeDef	TIM_BaseInitStructure;
	TIM_OCInitTypeDef		TIM_OCInitStructure;
	TIM_BDTRInitTypeDef		TIM_BDTRInitStructure;

/*时钟使能*/	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
/*初始化定时器的基础配置*/
	TIM_DeInit(TIM1);
	TIM_BaseInitStructure.TIM_ClockDivision			=	TIM_CKD_DIV1;							//时钟分割为1，即使用84MHz时钟
	TIM_BaseInitStructure.TIM_CounterMode			=	TIM_CounterMode_CenterAligned1;			//中心对齐模式1，在计数器递减时产生比较中断
																								//中心对齐模式2，在计数器递增时产生比较中断
																								//中心对齐模式3，在计数器递增或递减时产生比较中断
	TIM_BaseInitStructure.TIM_Period				=	4200-1;									//计数值溢出界限
	TIM_BaseInitStructure.TIM_Prescaler				=	1;										//预分频数为1
	TIM_BaseInitStructure.TIM_RepetitionCounter		=	0;
	TIM_TimeBaseInit(TIM1,&TIM_BaseInitStructure);
	
/*初始化定时器的各通道*/
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode					=	TIM_OCMode_PWM2;						//pwm模式1输出，向上计数时，cnt<ccr时为有效电平，向下计数时，cnt>ccr时为无效电平
																								//pwm模式2输出，向上计数时，cnt<ccr时为无效电平，向下计数时，cnt>ccr时为有效电平
	TIM_OCInitStructure.TIM_OCIdleState				=	TIM_OCIdleState_Reset;					//计数到pulse值后拉低管脚
	TIM_OCInitStructure.TIM_OCNIdleState			=	TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCPolarity				=	TIM_OCPolarity_Low;						//有效电平为低
	TIM_OCInitStructure.TIM_OCNPolarity				=	TIM_OCNPolarity_Low;					//互补输出通道，有效电平为低，与TIM_OCPolarity保持一致才能互补
	TIM_OCInitStructure.TIM_OutputState				=	TIM_OutputState_Enable;					//输出通道使能
	TIM_OCInitStructure.TIM_OutputNState			=	TIM_OutputNState_Enable;				//互补输出通道，输出使能
	TIM_OCInitStructure.TIM_Pulse					=	10;									//占空比设置
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_Pulse					=	10;
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_Pulse					=	10;
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);

/*开启计数器溢出中断，由于采用向上向下计数，当计数器到达Period值时，即是PWM高电平的中点位置，此时中断进行电流采样*/
	TIM_ClearFlag(TIM1,TIM_IT_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);

/*死区设置*/
	TIM_BDTRInitStructure.TIM_OSSRState				=	TIM_OSSRState_Disable;					//运行模式下输出选择
	TIM_BDTRInitStructure.TIM_OSSIState				=	TIM_OSSIState_Enable;					//空闲模式下输出选择
	TIM_BDTRInitStructure.TIM_LOCKLevel				=	TIM_LOCKLevel_OFF;						//锁定设置
	TIM_BDTRInitStructure.TIM_DeadTime				=	0x64;									//死区时间
	TIM_BDTRInitStructure.TIM_Break					=	TIM_Break_Disable;						//刹车功能使能
	TIM_BDTRInitStructure.TIM_BreakPolarity			=	TIM_BreakPolarity_High;					//刹车极性
	TIM_BDTRInitStructure.TIM_AutomaticOutput		=	TIM_AutomaticOutput_Enable;				//自动输出使能
	TIM_BDTRConfig(TIM1,&TIM_BDTRInitStructure);

	TIM_ARRPreloadConfig(TIM1,ENABLE);
	TIM_Cmd(TIM1,DISABLE);
	TIM_CtrlPWMOutputs(TIM1,DISABLE);
}

//CH1	无效电平为高--------------------------------------------------------------------------------------------------
	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH1_OFF_CH1N_ON(void)
	参数含义			：null
	函数功能			：上桥臂关		下桥臂开
	----------------------------------------------------------------------------*/
void	TIM1_CH1_OFF_CH1N_ON(void)
{
	TIM1->CCER &=0xFFFE;			//CCER->CC1E	= 0
	TIM1->CCER |=0x0004;			//CCER->CC1NE	= 1
	TIM1->CCMR1 &= 0xFF8F;
	TIM1->CCMR1 |= 0x0040;			//强制OC1输出无效电平
}

	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH1_OFF_CH1N_OFF(void)
	参数含义			：null
	函数功能			：上桥臂关		下桥臂关
	----------------------------------------------------------------------------*/
void	TIM1_CH1_OFF_CH1N_OFF(void)
{
	TIM1->CCER &=0xFFFE;			//CCER->CC1E	= 0
	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	= 0
	TIM1->CCMR1 &= 0xFF8F;
	TIM1->CCMR1 |= 0x0040;			//强制OC1输出无效电平
}

	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH1_ON_CH1N_OFF(void)
	参数含义			：null
	函数功能			：上桥臂开		下桥臂关
	----------------------------------------------------------------------------*/
void	TIM1_CH1_ON_CH1N_OFF(void)
{
	TIM1->CCER |=0x0001;			//CCER->CC1E	= 1
	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	= 0
	TIM1->CCMR1 &= 0xFF8F;
	TIM1->CCMR1 |= 0x0040;			//强制OC1输出无效电平
}

	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH1_OFF_CH1N_PWM(void)
	参数含义			：null
	函数功能			：上桥臂关		下桥臂pwm
	----------------------------------------------------------------------------*/
void	TIM1_CH1_OFF_CH1N_PWM(void)
{
	TIM1->CCMR1 &= 0xFF8F;
	TIM1->CCMR1 |= 0x0070;			//OC1为PWM2模式
	TIM1->CCER &=0xFFFE;			//CCER->CC1E	=	0
	TIM1->CCER |=0x0004;			//CCER->CC1NE	=	1
}


	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH1_PWM_CH1N_OFF(void)
	参数含义			：null
	函数功能			：上桥臂pwm		下桥臂关
	----------------------------------------------------------------------------*/
void	TIM1_CH1_PWM_CH1N_OFF(void)
{
	TIM1->CCMR1 &= 0xFF8F;
	TIM1->CCMR1 |= 0x0070;			//OC1为PWM2模式
	TIM1->CCER |=0x0001;			//CCER->CC1E	=	1
	TIM1->CCER &=0xFFFB;			//CCER->CC1NE	=	0
}

//CH2------------------------------------------------------------------------------------------------------------------
	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH2_OFF_CH2N_ON(void)
	参数含义			：null
	函数功能			：上桥臂关		下桥臂开
	----------------------------------------------------------------------------*/
void	TIM1_CH2_OFF_CH2N_ON(void)
{
	TIM1->CCER &=0xFFEF;			//CCER->CC2E	= 0
	TIM1->CCER |=0x0040;			//CCER->CC2NE	= 1
	TIM1->CCMR1 &= 0x8FFF;
	TIM1->CCMR1 |= 0x4000;			//强制OC1输出无效电平
}

	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH2_OFF_CH2N_OFF(void)
	参数含义			：null
	函数功能			：上桥臂关		下桥臂关
	----------------------------------------------------------------------------*/
void	TIM1_CH2_OFF_CH2N_OFF(void)
{
	TIM1->CCER &=0xFFEF;			//CCER->CC2E	= 0
	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	= 0
	TIM1->CCMR1 &= 0x8FFF;
	TIM1->CCMR1 |= 0x4000;			//强制OC1输出无效电平
}

	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH2_ON_CH2N_OFF(void)
	参数含义			：null
	函数功能			：上桥臂开		下桥臂关
	----------------------------------------------------------------------------*/
void	TIM1_CH2_ON_CH2N_OFF(void)
{
	TIM1->CCER |=0x0010;			//CCER->CC2E	= 1
	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	= 0
	TIM1->CCMR1 &= 0x8FFF;
	TIM1->CCMR1 |= 0x4000;			//强制OC2输出无效电平
}

	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH2_OFF_CH2N_PWM(void)
	参数含义			：null
	函数功能			：上桥臂关		下桥臂pwm
	----------------------------------------------------------------------------*/
void	TIM1_CH2_OFF_CH2N_PWM(void)
{
	TIM1->CCMR1 &= 0x8FFF;
	TIM1->CCMR1 |= 0x7000;			//OC2为PWM2模式
	TIM1->CCER &=0xFFEF;			//CCER->CC2E	=	0
	TIM1->CCER |=0x0040;			//CCER->CC2NE	=	1
}


	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH2_PWM_CH2N_OFF(void)
	参数含义			：null
	函数功能			：上桥臂pwm		下桥臂关
	----------------------------------------------------------------------------*/
void	TIM1_CH2_PWM_CH2N_OFF(void)
{
	TIM1->CCMR1 &= 0x8FFF;
	TIM1->CCMR1 |= 0x7000;			//OC2为PWM2模式
	TIM1->CCER |=0x0010;			//CCER->CC2E	=	1
	TIM1->CCER &=0xFFBF;			//CCER->CC2NE	=	0
}
//CH3-------------------------------------------------------------------------------------------------------------------------
	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH3_OFF_CH3N_ON(void)
	参数含义			：null
	函数功能			：上桥臂关		下桥臂开
	----------------------------------------------------------------------------*/
void	TIM1_CH3_OFF_CH3N_ON(void)
{
	TIM1->CCER &=0xFEFF;			//CCER->CC3E	= 0
	TIM1->CCER |=0x0400;			//CCER->CC3NE	= 1
	TIM1->CCMR2 &= 0xFF8F;
	TIM1->CCMR2 |= 0x0040;			//强制OC3输出无效电平
}

	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH3_OFF_CH3N_OFF(void)
	参数含义			：null
	函数功能			：上桥臂关		下桥臂关
	----------------------------------------------------------------------------*/
void	TIM1_CH3_OFF_CH3N_OFF(void)
{
	TIM1->CCER &=0xFEFF;			//CCER->CC3E	= 0
	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	= 0
	TIM1->CCMR2 &= 0xFF8F;
	TIM1->CCMR2 |= 0x0040;			//强制OC3输出无效电平
}

	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH3_ON_CH3N_OFF(void)
	参数含义			：null
	函数功能			：上桥臂开		下桥臂关
	----------------------------------------------------------------------------*/
void	TIM1_CH3_ON_CH3N_OFF(void)
{
	TIM1->CCER |=0x0100;			//CCER->CC3E	= 1
	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	= 0
	TIM1->CCMR2 &= 0xFF8F;
	TIM1->CCMR2 |= 0x0040;			//强制OC3输出无效电平
}

	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH3_OFF_CH3N_PWM(void)
	参数含义			：null
	函数功能			：上桥臂关		下桥臂pwm
	----------------------------------------------------------------------------*/
void	TIM1_CH3_OFF_CH3N_PWM(void)
{
	TIM1->CCMR2 &= 0xFF8F;
	TIM1->CCMR2 |= 0x0070;			//OC3为PWM2模式
	TIM1->CCER &=0xFEFF;			//CCER->CC3E	=	0
	TIM1->CCER |=0x0400;			//CCER->CC3NE	=	1
}


	/*---------------------------------------------------------------------------
	函数名称			：TIM1_CH3_PWM_CH3N_OFF(void)
	参数含义			：null
	函数功能			：上桥臂pwm		下桥臂关
	----------------------------------------------------------------------------*/
void	TIM1_CH3_PWM_CH3N_OFF(void)
{
	TIM1->CCMR2 &= 0xFF8F;
	TIM1->CCMR2 |= 0x0070;			//OC3为PWM2模式
	TIM1->CCER |=0x0100;			//CCER->CC3E	=	1
	TIM1->CCER &=0xFBFF;			//CCER->CC3NE	=	0
}


	/*---------------------------------------------------------------------------
	函数名称			：Timer3_Config(void)
	参数含义			：null
	函数功能			：用于接收光电编码器的信号			timer3 APB1上42MHz
	----------------------------------------------------------------------------*/
void	Timer3_Config(void)
{
	TIM_TimeBaseInitTypeDef 		TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef				TIM_ICInitStructure;	
	
/*时钟使能*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
/*Timer3 配置*/
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_CounterMode			=	TIM_CounterMode_Up;					//向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision			=	TIM_CKD_DIV1;						//TIM3时钟分割，使用42MHz频率
	TIM_TimeBaseInitStructure.TIM_Period				=	10;								//当脉冲到达此值时，重新装载，并产生一次中断
	TIM_TimeBaseInitStructure.TIM_Prescaler				=	0;									//
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
/*设置定时器为编码器模式，IT1,IT2为上升沿计数*/
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel						=	TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter					=	1;									//输入滤波器，超过1个时钟计数周期算有效
	TIM_ICInit(TIM3,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel						=	TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter					=	1;									//输入滤波器，超过1个时钟计数周期算有效
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM3->CNT = 0;
	TIM_Cmd(TIM3,ENABLE);	
}


	/*---------------------------------------------------------------------------
	函数名称			：Timer4_Config(void)
	参数含义			：null
	函数功能			：	用于检测霍尔传感器状态			timer4 APB1上42MHz
							16bit定时器
							TIM4的CH1/CH2/CH3配置为输入捕获，在CH1处进行三通道异或，
							当三个通道信号有变时，会触发TIM4的中断，在中断函数中进行
							换向操作。
	----------------------------------------------------------------------------*/
void	Timer4_Config(void)
{
	TIM_TimeBaseInitTypeDef	TIM_BaseInitStructure;
	TIM_ICInitTypeDef		TIM_ICInitStructure;

/*时钟开启*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
/*定时器基础设置*/
	TIM_BaseInitStructure.TIM_ClockDivision			=	0;									//时钟分频系数1，使用42Mhz时钟频率
	TIM_BaseInitStructure.TIM_CounterMode			=	TIM_CounterMode_Up;					//向上计数模式
	TIM_BaseInitStructure.TIM_Period				=	65535;								//计数最大值
	TIM_BaseInitStructure.TIM_Prescaler				=	0;									//预分频系数1，使用42MHz
	TIM_TimeBaseInit(TIM4,&TIM_BaseInitStructure);

/*通道输入设置*/	
	TIM_ICInitStructure.TIM_Channel					=	TIM_Channel_1;						//通道一
	TIM_ICInitStructure.TIM_ICPolarity				=	TIM_ICPolarity_BothEdge;			//捕获上升沿与下降沿
	TIM_ICInitStructure.TIM_ICFilter				=	0;									//不进行滤波
	TIM_ICInitStructure.TIM_ICPrescaler				=	0;									//输入信号不分频
	TIM_ICInitStructure.TIM_ICSelection				=	TIM_ICSelection_DirectTI;			//通道与TIx直连
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel					=	TIM_Channel_2;
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel					=	TIM_Channel_3;
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
/*启用霍尔传感器模式，即IC1/IC2/IC3连到TI，亦或组合*/	
	TIM_SelectHallSensor(TIM4,ENABLE);
	
/*开启捕获中断*/
//	TIM_ClearFlag(TIM4,TIM_IT_CC1);
//	TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);													//使能更新中断
	TIM_Cmd(TIM4,DISABLE);																	//开启定时器4
}


	/*---------------------------------------------------------------------------
	函数名称			：ADC_DMA_Config(void)
	参数含义			：null
	函数功能			：ADC引脚与DMA初始化
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

	/*DMA 配置，使用DMA2_Channel_0 Stream_0*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel					=	DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr		=	(uint32_t)(&ADC1->DR);					//使用ADC3的数据寄存器作为DMA的外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr			=	(uint32_t)(&(m_motor_rt_para.ADC_DMA_buf));//将数组首地址作为内存地址
	DMA_InitStructure.DMA_DIR						=	DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize				=	32;
	DMA_InitStructure.DMA_PeripheralInc				=	DMA_PeripheralInc_Disable;				//DMA传输过程中外设地址不增长
	DMA_InitStructure.DMA_MemoryInc					=	DMA_MemoryInc_Enable;					//DMA传输过程中内存地址增长
	DMA_InitStructure.DMA_PeripheralDataSize		=	DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize			=	DMA_MemoryDataSize_HalfWord;			//使用16位数据作为传输单位
	DMA_InitStructure.DMA_Mode						=	DMA_Mode_Circular;						//设置DMA传输模式为循环传输
	DMA_InitStructure.DMA_Priority					=	DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode					=	DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold				=	DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst				=	DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst			=	DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0,ENABLE);																//使能ADC的DMA

/*ADC1 配置*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);										//ADC挂载在APB2上
	
	ADC_DeInit();
	
	ADC_CommonInitStructure.ADC_DMAAccessMode		=	ADC_DMAAccessMode_Disabled;				//
	ADC_CommonInitStructure.ADC_Mode				=	ADC_Mode_Independent;					//只使用一路ADC，无须同步，所以用独立模式即可
	/*ADC的时钟设为 84MHz/8 =10.5MHz*/
	ADC_CommonInitStructure.ADC_Prescaler			=	ADC_Prescaler_Div4;						//APB2总线时钟频率为84MHz，可设置分频系数2/4/6/8，ADC的时钟需要设置在14MHz以下才可以正常使用??
	/*电流环更新为20KHz，电压值采样需要高于320KHz，每隔32次时钟采一次样*/
	ADC_CommonInitStructure.ADC_TwoSamplingDelay	=	ADC_TwoSamplingDelay_5Cycles;			//相邻两次采样时钟间隔，可设置为5-20，每通道转化时间=采样时间+12个ADC时钟周期=15个时钟周期
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	ADC_InitStructure.ADC_Resolution				=	ADC_Resolution_12b;						//采样分辨率为12位
	ADC_InitStructure.ADC_ScanConvMode				=	ENABLE;									//扫描模式(多通道ADC采集要用扫描模式)
	ADC_InitStructure.ADC_ContinuousConvMode		=	ENABLE;									//关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConv			=	ADC_ExternalTrigConvEdge_None;			//没有外部触发
	ADC_InitStructure.ADC_DataAlign					=	ADC_DataAlign_Right;					//数据格式右对齐
	ADC_InitStructure.ADC_NbrOfConversion			=	4;										//仅使用6个通道
	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_6,1,ADC_SampleTime_15Cycles);						//规则模式通道设置，电流传感器电压
	ADC_RegularChannelConfig(ADC1,ADC_Channel_7,2,ADC_SampleTime_15Cycles);						//规则模式通道设置，母线电压
	ADC_RegularChannelConfig(ADC1,ADC_Channel_14,3,ADC_SampleTime_15Cycles);					//规则模式通道设置，压力传感器1电压
	ADC_RegularChannelConfig(ADC1,ADC_Channel_15,4,ADC_SampleTime_15Cycles);					//规则模式通道设置，压力传感器2电压

/*DMA中断配置*/
	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TC);
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);												//设置DMA传输完成中断
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);											//使能DMA传输完成后请求ADC
	ADC_DMACmd(ADC1,ENABLE);																	//使能ADC的DMA	
	ADC_Cmd(ADC1,ENABLE);																		//使能ADC1
	ADC_SoftwareStartConv(ADC1);																//开启ADC1的软件转换
}


	/*---------------------------------------------------------------------------
	函数名称			：CAN_Config(void)
	参数含义			：null
	函数功能			：
	----------------------------------------------------------------------------*/
void	CAN_Config(void)
{

}


	/*---------------------------------------------------------------------------
	函数名称			：BrakeControl(void)
	参数含义			：mode:		1 开启抱闸		0 关闭抱闸		其它 无操作
	函数功能			：开启关闭抱闸使用函数
	----------------------------------------------------------------------------*/
void	BrakeControl(uint8_t	mode)
{
	uint8_t 	cnt = 0;
	uint32_t	io_state;
	if(mode == 0)
	{
		GPIOC->ODR |= 0x2000;											//PC13置高
		//继电器上电后 PC14 -> 0	PC15 -> 1 指示状态
//		while(1)
//		{
//			io_state	=	(GPIOC->IDR >> 14) & 0xFFFC;
//			if(io_state == 0x02)
//				break;
//			else cnt++;
//			delay_us(2000);
//			if(cnt >= 250)												//超过0.5s没有确认抱闸打开，认为抱闸打开错误
//			{
//				m_error.brake_err = open_error;
//				return;
//			}
//		}
	}
	else if(mode == 1)
	{
		GPIOC->ODR &= 0xDFFF;											//PC13置低
		//继电器断电后 PC14 -> 1	PC15 -> 0 指示状态
//		while(1)
//		{
//			io_state	=	(GPIOC->IDR >> 14) & 0xFFFC;
//			if(io_state == 0x01)
//				break;
//			else cnt++;
//			delay_us(2000);
//			if(cnt >= 250)												//超过0.5s没有确认抱闸打开，认为抱闸打开错误
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
	函数名称			：LED_ON(void)
	参数含义			：
	函数功能			：LED点亮
	----------------------------------------------------------------------------*/
void	LED_ON(void)
{
	GPIOB->ODR |= 0x0200;
}

	/*---------------------------------------------------------------------------
	函数名称			：LED_OFF(void)
	参数含义			：
	函数功能			：LED熄灭
	----------------------------------------------------------------------------*/
void	LED_OFF(void)
{
	GPIOB->ODR &= 0xFDFF;
}

/**********************end of file************************/
