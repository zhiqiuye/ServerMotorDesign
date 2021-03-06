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
#include	"stm32f4xx_dac.h"
#include	"ucos_ii.h"
#include	"includes.h"
#include	"global_parameters.h"
#include	"delay.h"
#include	"motor_control.h"

uint8_t		wdata_temp[5]	=	{0xAA,0xAA,0xAA,0xAA,0xAA};

	/*---------------------------------------------------------------------------
	函数名称			：NVIC_Config(void)
	参数含义			：null
	函数功能			：中断向量组优先级分配
							中断向量组选择2
							包含4级抢占式中断优先级		0-3
							以及4级响应优先级			0-3
	
							TIM1_UP			pwm计数溢出中断	20KHz
							TIM2			电流环定时中断  20KHz
							TIM3			增量编码器捕获计数溢出中断	
							TIM4			霍尔传感器捕获中断
							TIM8			速度环位置环更新
							DMA2_Stream0	ADC传输完成中断
	----------------------------------------------------------------------------*/
void	NVIC_Config(void)
{
	NVIC_InitTypeDef 	NVIC_InitStructure;
	
	/*PWM产生，电流环更新以及定时触发ADC采样，TIM1 计数溢出中断*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/*Hall 传感器输入捕获中断，TIM4输入捕获中断*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);	
	
	/*ADC-DMA 传输完成中断，相电流检测*/
	NVIC_InitStructure.NVIC_IRQChannel						=	DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	2;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/*ADC-DMA 传输完成中断，力矩传感器检测*/
	NVIC_InitStructure.NVIC_IRQChannel						=	DMA2_Stream3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);	
	
	/*速度环、位置环更新中断，TIM2 计数溢出中断*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	3;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/*TIM8 速度位置环输入时钟源*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM8_UP_TIM13_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/*CAN BUS 接收发送中断初始化*/
	NVIC_InitStructure.NVIC_IRQChannel						=	CAN1_RX0_IRQn;			//FIFO0的接收中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;						//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1;						//
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;					//
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel						=	CAN1_TX_IRQn;			//FIFO0的发送中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;						//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	2;						//
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;					//
	NVIC_Init(&NVIC_InitStructure);	

	/*SPI 1 DMA RX中断*/
	NVIC_InitStructure.NVIC_IRQChannel						=	DMA2_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;						//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	3;						//响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);

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

/*LED管脚配置 PC1/2*/	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_2MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_1|GPIO_Pin_2);
	
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

/*增量式编码器T法测速端口*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_0;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);

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
	
/*AD模拟量采集，三相电流采集使用 PA1/2/3,20180914*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	
/*AD模拟量采集，力矩传感器采集使用 PC4/5*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
	
/*AD模拟量采集，母线电压采集使用 PC3*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_3;
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
	
/*CAN总线接口*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
	
/*SPI 1 端口初始化 读取绝对值编码器数据***********/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_NOPULL;//DOWN;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_3|GPIO_Pin_5;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_DOWN;//NOPULL;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_4;
	GPIO_Init(GPIOB,&GPIO_InitStructure);	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI1);
	
/*test 20180739 pc 0*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_0;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_0);
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
	函数功能			：	SPI1使用DMA方式通讯
							SPI1	-	CLK		-	PB3
							SPI1	-	MISO	-	PB4

							SPI1 TX	-	DMA2 Stream3 Ch3
							SPI1 RX	-	DMA2 Stream2 Ch3
	----------------------------------------------------------------------------*/
void	SPI1_DMA_Config(void)
{
	SPI_InitTypeDef		SPI_InitStructure;
	DMA_InitTypeDef		DMA_InitStructure;
	
/*时钟配置************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
/*SPI 1 配置************/
	SPI_I2S_DeInit(SPI1);
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_BaudRatePrescaler	=	SPI_BaudRatePrescaler_256;							//外设时钟84MHz
	SPI_InitStructure.SPI_Direction			=	SPI_Direction_2Lines_FullDuplex;//SPI_Direction_1Line_Rx;//
	SPI_InitStructure.SPI_Mode				=	SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize			=	SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPHA				=	SPI_CPHA_2Edge;										//第二个跳变沿读取数据
	SPI_InitStructure.SPI_CPOL				=	SPI_CPOL_High;										//总线空闲为高电平
	SPI_InitStructure.SPI_NSS				=	SPI_NSS_Soft;										//软件控制片选信号
	SPI_InitStructure.SPI_FirstBit			=	SPI_FirstBit_MSB;									//高位在前
	SPI_Init(SPI1,&SPI_InitStructure);
	SPI_CalculateCRC(SPI1,DISABLE);																	//关闭CRC校验
	SPI_Cmd(SPI1,ENABLE);																			//使能SPI1
	
/*SPI DMA Init*/
	DMA_DeInit(DMA2_Stream2);
	DMA_DeInit(DMA2_Stream3);
	
/*Rx SPI1 DMA通道初始化*/
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel				=	DMA_Channel_3;										//DMA通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr	=	(uint32_t)(&SPI1->DR);								//DMA外设地址为SPI的数据寄存器
	DMA_InitStructure.DMA_Memory0BaseAddr		=	(uint32_t)(&m_motor_rt_para.m_abs_encoder.u8_abs_raw_data[0]);//DMA的内存地址设置
	DMA_InitStructure.DMA_DIR					=	DMA_DIR_PeripheralToMemory;							//DMA方向为从外设寄存器到内存
	DMA_InitStructure.DMA_BufferSize			=	4;													//DMA的Buffer设置大小，实际使用时再做改动
	DMA_InitStructure.DMA_PeripheralInc			=	DMA_PeripheralInc_Disable;							//外设地址不变
	DMA_InitStructure.DMA_MemoryInc				=	DMA_MemoryInc_Enable;								//内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;						//外设数据宽度为8bits
	DMA_InitStructure.DMA_MemoryDataSize		=	DMA_MemoryDataSize_Byte;							//内存数据宽为8bits
	DMA_InitStructure.DMA_Mode					=	DMA_Mode_Normal;									//
	DMA_InitStructure.DMA_Priority				=	DMA_Priority_High;									//DMA优先级别
	DMA_InitStructure.DMA_FIFOMode				=	DMA_FIFOMode_Disable;								//不用fifo模式，使用直接模式
	DMA_InitStructure.DMA_FIFOThreshold			=	DMA_FIFOThreshold_Full;								//fifo的阈值
	DMA_InitStructure.DMA_MemoryBurst			=	DMA_MemoryBurst_Single;								//内存单次传输数据宽度，每次转移一个数据
	DMA_InitStructure.DMA_PeripheralBurst		=	DMA_PeripheralBurst_Single;							//外设单次传输数据宽度，每次转移一个数据
	DMA_Init(DMA2_Stream2,&DMA_InitStructure);
	
/*Tx SPI1 DMA通道初始化*/
	DMA_InitStructure.DMA_Channel				=	DMA_Channel_3;
	DMA_InitStructure.DMA_PeripheralBaseAddr	=	(uint32_t)(&SPI1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr		=	(uint32_t)(&wdata_temp[0]);
	DMA_InitStructure.DMA_DIR					=	DMA_DIR_MemoryToPeripheral;	
	DMA_InitStructure.DMA_BufferSize			=	4;
	DMA_InitStructure.DMA_Priority				=	DMA_Priority_Low;
	DMA_Init(DMA2_Stream3,&DMA_InitStructure);
	
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,ENABLE);
	
/*SPI DMA Rx中断配置*/	
	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC | DMA_IT_TE, ENABLE);
/*SPI DMA Tx中断配置*/
	DMA_ITConfig(DMA2_Stream3, DMA_IT_TC | DMA_IT_TE, ENABLE);

	DMA_Cmd(DMA2_Stream2,DISABLE);
	DMA_Cmd(DMA2_Stream3,DISABLE);	
	DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);
	DMA_ClearITPendingBit(DMA2_Stream3,DMA_IT_TCIF3);
}


	/*---------------------------------------------------------------------------
	函数名称			：	SPI_DMA_ReadData
	参数含义			：	length			读取数据长度
	函数功能			：	通过SPI DMA方式读取绝对编码器位置
							读取数据存储在readBuf[1]开始的地方，readBuf[0]中不是
	----------------------------------------------------------------------------*/
void SPI_DMA_ReadData(uint16_t length)
{
	DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);
	DMA_ClearITPendingBit(DMA2_Stream3,DMA_IT_TCIF3);
	DMA2_Stream2->NDTR		=	length;
	DMA2_Stream2->M0AR		=	(uint32_t)&m_motor_rt_para.m_abs_encoder.u8_abs_raw_data[0];
	DMA2_Stream3->NDTR		=	length;
	DMA2_Stream3->M0AR		=	(uint32_t)&wdata_temp[0];
	DMA_Cmd(DMA2_Stream2,ENABLE);
	DMA_Cmd(DMA2_Stream3,ENABLE);
}



void SPI_DMA_WriteData(uint16_t length)
{
	DMA_ClearITPendingBit(DMA2_Stream3,DMA_IT_TCIF3);
	DMA2_Stream3->NDTR		=	length;
	DMA2_Stream3->M0AR		=	(uint32_t)&wdata_temp[0];
	DMA_Cmd(DMA2_Stream3,ENABLE);
}

	/*PWM产生，电流环更新--------------------------------------------------------
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
	TIM_BaseInitStructure.TIM_Prescaler				=	0;										//预分频数为1
	TIM_BaseInitStructure.TIM_RepetitionCounter		=	0;
	TIM_TimeBaseInit(TIM1,&TIM_BaseInitStructure);
	
/*初始化定时器的各通道*/
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode					=	TIM_OCMode_PWM2;						//pwm模式1输出，向上计数时，cnt<ccr时为有效电平，向下计数时，cnt>ccr时为无效电平																								//pwm模式2输出，向上计数时，cnt<ccr时为无效电平，向下计数时，cnt>ccr时为有效电平
	TIM_OCInitStructure.TIM_OCIdleState				=	TIM_OCIdleState_Reset;//TIM_OCIdleState_Reset;					//计数到pulse值后拉低管脚
	TIM_OCInitStructure.TIM_OCNIdleState			=	TIM_OCNIdleState_Reset;//TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCPolarity				=	TIM_OCPolarity_Low;//TIM_OCPolarity_Low;						//有效电平为低
	TIM_OCInitStructure.TIM_OCNPolarity				=	TIM_OCNPolarity_Low;//TIM_OCNPolarity_Low;					//互补输出通道，有效电平为低，与TIM_OCPolarity保持一致才能互补
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
	TIM_BDTRInitStructure.TIM_DeadTime				=	0xE0;									//死区时间，中间对齐模式下，前后加起来死区时间是N/84 us,IR2130S的死区时间为2us，具体计算，参见BDTR寄存器设置
	TIM_BDTRInitStructure.TIM_Break					=	TIM_Break_Disable;						//刹车功能使能
	TIM_BDTRInitStructure.TIM_BreakPolarity			=	TIM_BreakPolarity_High;					//刹车极性
	TIM_BDTRInitStructure.TIM_AutomaticOutput		=	TIM_AutomaticOutput_Enable;				//自动输出使能
	TIM_BDTRConfig(TIM1,&TIM_BDTRInitStructure);

/*强制H1-3,L1-3输出低*/
	IR2130S_force_reset();
	TIM_ARRPreloadConfig(TIM1,ENABLE);															//使能自动重装载寄存器
	TIM_Cmd(TIM1,ENABLE);																		//禁用定时器1
	TIM_CtrlPWMOutputs(TIM1,ENABLE);															//使能timer1输出PWM
}


	/*速度位置环更新-------------------------------------------------------------
	函数名称			：Timer2_Config(void)
	参数含义			：null
	函数功能			：	产生1KHz频率中断，进行速度位置环的PI调节
							TIM2时钟作为从定时器使用，其主定时器为TIM8，
							TIM8产生10KHz时钟信号，
							要更新位置环速度环，需要打开TIM2
	----------------------------------------------------------------------------*/
void	Timer2_Config(void)
{
	TIM_TimeBaseInitTypeDef 		TIM_TimeBaseInitStructure;

/*时钟配置*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
/*Timer2配置*/
	TIM_DeInit(TIM2);																			//Timer 2 在APB1上，42MHz，在system_stm32f4xx.c中
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_ClockDivision				=	TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode				=	TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period					=	5-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler					=	2-1;			
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_SelectSlaveMode(TIM2,TIM_SlaveMode_Trigger);											//TIM2选择从模式的触发模式
	TIM_SelectInputTrigger(TIM2,TIM_TS_ITR1);													//选择TIM8的输出信号作为时钟源
	TIM_ITRxExternalClockConfig(TIM2,TIM_TS_ITR1);												//选择外部时钟触发

	TIM_Cmd(TIM2,DISABLE);
	TIM_ClearFlag(TIM2,TIM_IT_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
}


	/*光电增量编码器-------------------------------------------------------------
	函数名称			：Timer3_Config(void)
	参数含义			：null
	函数功能			：用于接收光电编码器的信号			timer3 APB1上42MHz
							A相		TIM3->Channel1
							B相		TIM3->Channel2
							C相		TIM3 输入捕获
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
	TIM_TimeBaseInitStructure.TIM_Period				=	65535;								//当脉冲到达此值时，重新装载，并产生一次中断
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
	
/*输入通道设置*/
	TIM_ICInitStructure.TIM_Channel						=	TIM_Channel_3;						//编码器I相输入
	TIM_ICInitStructure.TIM_ICPolarity					=	TIM_ICPolarity_Rising;				//只捕获上升沿
	TIM_ICInitStructure.TIM_ICSelection					=	TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICFilter					=	0;
	TIM_ICInitStructure.TIM_ICPrescaler					=	0;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
	
//	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
//	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC3,ENABLE);
	TIM3->CNT = 0;
	TIM_Cmd(TIM3,ENABLE);	
}


	/*霍尔中断信号---------------------------------------------------------------
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
	TIM_ClearFlag(TIM4,TIM_IT_CC1);
	TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);													//使能更新中断
	TIM_Cmd(TIM4,ENABLE);																	//开启定时器4
}




	/*低速T法测速----------------------------------------------------------------
	函数名称			：	Timer5_Config(void)
	参数含义			：	null
	函数功能			：	用于高速端光电编码器低速值测量，即测量脉宽
							TIM5 CH1输入编码器PWM输入模式
							TIM5 挂在APB1上，是其时钟频率42MHz的两倍，

							记录的脉冲周期值通过DMA传送给响应内存地址，
							使用DMA通道：DMA1 stream 1 channel 6



							低速测量最低周期  1.56ms
	----------------------------------------------------------------------------*/
void	Timer5_Config(void)
{
	TIM_TimeBaseInitTypeDef 		TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef				TIM_ICInitStructure;	
	DMA_InitTypeDef					DMA_InitStructure;
	
/*时钟使能*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE );

/*dma配置*/
	DMA_DeInit(DMA1_Stream2);
	DMA_InitStructure.DMA_Channel					=	DMA_Channel_6;
	DMA_InitStructure.DMA_PeripheralBaseAddr		=	(uint32_t)(&TIM5->CCR1);				//使用TIM5 CCR1的数据寄存器作为DMA的外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr			=	(uint32_t)(&(m_motor_rt_para.m_inc_encoder.u32_pulse_width_buf));//将数组首地址作为内存地址
	DMA_InitStructure.DMA_DIR						=	DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize				=	PW_BUFFER_LENGTH;						//
	DMA_InitStructure.DMA_PeripheralInc				=	DMA_PeripheralInc_Disable;				//DMA传输过程中外设地址不增长
	DMA_InitStructure.DMA_MemoryInc					=	DMA_MemoryInc_Enable;					//DMA传输过程中内存地址增长
	DMA_InitStructure.DMA_PeripheralDataSize		=	DMA_PeripheralDataSize_Word;			//使用16位数据作为传输单位，尽管TIM5->CCR1为32位寄存器，但是ARR为16位，
	DMA_InitStructure.DMA_MemoryDataSize			=	DMA_MemoryDataSize_Word;				//
	DMA_InitStructure.DMA_Mode						=	DMA_Mode_Circular;						//设置DMA传输模式为循环传输
	DMA_InitStructure.DMA_Priority					=	DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode					=	DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold				=	DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst				=	DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst			=	DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream2,&DMA_InitStructure);
	DMA_Cmd(DMA1_Stream2,ENABLE);																//使能ADC的DMA
	
	
/*Timer5 配置*/
	TIM_DeInit(TIM5);
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_CounterMode			=	TIM_CounterMode_Up;					//向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision			=	TIM_CKD_DIV1;						//TIM5时钟分割，
	TIM_TimeBaseInitStructure.TIM_Period				=	0xFFFFFFFF;							//当脉冲到达此值时，重新装载，并产生一次中断，意味着获取的脉冲周期在65535个时钟周期以内
	TIM_TimeBaseInitStructure.TIM_Prescaler				=	1;									//2倍预分频，使用42MHz频率
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
/*设置定时器输入捕获通道*/	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel						=	TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity					=	TIM_ICPolarity_Rising;				//触发捕获的电平，下降沿捕获
	TIM_ICInitStructure.TIM_ICSelection					=	TIM_ICSelection_DirectTI;			//映射到TI1上
	TIM_ICInitStructure.TIM_ICPrescaler					=	TIM_ICPSC_DIV1;						//输入捕获分频，此处是每个脉冲都捕捉
	TIM_ICInitStructure.TIM_ICFilter					=	0;									//输入滤波器，每个脉冲都有效
	TIM_PWMIConfig(TIM5,&TIM_ICInitStructure);
	
	TIM_SelectInputTrigger(TIM5,TIM_TS_TI1FP1);													//输入跳变选择，使用IN1信号作为输入
	TIM_SelectSlaveMode(TIM5,TIM_SlaveMode_Reset);												//TIM从机模式：触发信号的上升沿重新初始化计数器和触发寄存器的更新事件
	TIM_SelectMasterSlaveMode(TIM5,TIM_MasterSlaveMode_Enable);									//启动定时器被动触发功能
	TIM_DMAConfig(TIM5,TIM_DMABase_CCR1,TIM_DMABurstLength_2Bytes);								//配置定时器的DMA，每次读寄存器是2bytes
	TIM_DMACmd(TIM5,TIM_DMA_CC1,ENABLE);														//使能CCR1 dma请求
	
	TIM5->CNT = 0;
	TIM_Cmd(TIM5,ENABLE);	
}

	/*电流环速度环同步定时，读数绝对值编码器-------------------------------------------------------
	函数名称			：	Timer8_Config(void)
	参数含义			：	null
	函数功能			：	用于 定时触发spi dma，
							TIM8 挂在APB2上，时钟为168Mhz
							工作在主模式，10KHz触发TIM2（速度位置环更新）作为从定时器
							TIM8触发信号为ITR0
							在定时器中断中开启spi dma读取信号，获取ssi信号值的时间在
							200us左右
	----------------------------------------------------------------------------*/
void	Timer8_Master_Config(void)
{
	TIM_TimeBaseInitTypeDef	TIM_BaseInitStructure;
	TIM_ICInitTypeDef		TIM_ICInitStructure;

/*时钟开启*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	
/*定时器基础设置*/
	TIM_BaseInitStructure.TIM_ClockDivision			=	TIM_CKD_DIV1;						//时钟分频系数1，使用168Mhz时钟频率
	TIM_BaseInitStructure.TIM_CounterMode			=	TIM_CounterMode_Up;					//向上计数模式
	TIM_BaseInitStructure.TIM_Period				=	840 - 1;							//计数最大值
	TIM_BaseInitStructure.TIM_Prescaler				=	20 - 1;								//预分频系数2，使用84MHz
	TIM_TimeBaseInit(TIM8,&TIM_BaseInitStructure);

	TIM_SelectOutputTrigger(TIM8,TIM_TRGOSource_Update);									//选择TIM9的级联输出信号为TRGO
	TIM_SelectMasterSlaveMode(TIM8,TIM_MasterSlaveMode_Enable);								//打开TIM9的级联模式
	
/*TIM9中断配置，定时器溢出中断*/
	TIM_ClearFlag(TIM8,TIM_IT_Update);
	TIM_ITConfig(TIM8,TIM_IT_Update,ENABLE);												//使能更新中断
	TIM_Cmd(TIM8,DISABLE);
}



	/*电流环检测DMA2方式----------------------------------------------------------
	函数名称			：ADC1_DMA_Config(void)
	参数含义			：null
	函数功能			：ADC引脚与DMA初始化
							PA1				ADC123_IN_1			U相电流
							PA2				ADC123_IN_2			V相电流
							PA3				ADC123_IN_3			W相电流
							ADC1	DMA2  stream 0 channel 0
	----------------------------------------------------------------------------*/
void	ADC1_DMA_Config(void)
{
	ADC_InitTypeDef			ADC_InitStructure;
	ADC_CommonInitTypeDef	ADC_CommonInitStructure;
	DMA_InitTypeDef			DMA_InitStructure;

/*ADC1 配置*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);										//ADC1挂载在APB2上
	
	ADC_DeInit();
	
	ADC_CommonInitStructure.ADC_DMAAccessMode		=	ADC_DMAAccessMode_Disabled;				//
	ADC_CommonInitStructure.ADC_Mode				=	ADC_Mode_Independent;					//只使用一路ADC，无须同步，所以用独立模式即可
	/*ADC的时钟设为 84MHz/8 =10.5MHz*/
	ADC_CommonInitStructure.ADC_Prescaler			=	ADC_Prescaler_Div4;						//APB2总线时钟频率为84MHz，可设置分频系数2/4/6/8，ADC的时钟需要设置在14MHz以下才可以正常使用??
	/*电流环更新为20KHz，电压值采样需要高于320KHz，每隔32次时钟采一次样*/
	ADC_CommonInitStructure.ADC_TwoSamplingDelay	=	ADC_TwoSamplingDelay_8Cycles;			//相邻两次采样时钟间隔，可设置为5-20，每通道转化时间=采样时间+12个ADC时钟周期=15个时钟周期
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	ADC_InitStructure.ADC_Resolution				=	ADC_Resolution_12b;						//采样分辨率为12位
	ADC_InitStructure.ADC_ScanConvMode				=	ENABLE;									//扫描模式(多通道ADC采集要用扫描模式)
	ADC_InitStructure.ADC_ContinuousConvMode		=	DISABLE;								//关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge		=	ADC_ExternalTrigConvEdge_None;			//没有外部触发,使用软件触发
	ADC_InitStructure.ADC_DataAlign					=	ADC_DataAlign_Right;					//数据格式右对齐
	ADC_InitStructure.ADC_NbrOfConversion			=	3;										//仅使用3个通道
	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_15Cycles);						//规则模式通道设置，U
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,2,ADC_SampleTime_15Cycles);						//规则模式通道设置，V
	ADC_RegularChannelConfig(ADC1,ADC_Channel_3,3,ADC_SampleTime_15Cycles);						//规则模式通道设置，W

/*DMA 配置，使用DMA2_Channel_0 Stream_0*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel					=	DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr		=	(uint32_t)(&ADC1->DR);					//使用ADC3的数据寄存器作为DMA的外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr			=	(uint32_t)(&(m_motor_rt_para.m_current_sensor.ADC_DMA_buf));//将数组首地址作为内存地址
	DMA_InitStructure.DMA_DIR						=	DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize				=	3;										//buffer size太大，采样时间过长，采样一次很快
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
	
/*DMA中断配置*/
	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TC);
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);												//设置DMA传输完成中断
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);											//使能DMA传输完成后请求ADC
	ADC_DMACmd(ADC1,ENABLE);																	//使能ADC的DMA	
	ADC_Cmd(ADC1,ENABLE);																		//使能ADC1
	ADC_SoftwareStartConv(ADC1);																//开启ADC1的软件转换
}



	/*力矩传感器检测DMA2方式----------------------------------------------------------
	函数名称			：ADC2_DMA_Config(void)
	参数含义			：null
	函数功能			：ADC引脚与DMA初始化
							PC4				ADC12_IN_14			T_sen_1
							PC5				ADC12_IN_15			T_sen_2							
							ADC2	DMA2  stream 3 channel 1
	----------------------------------------------------------------------------*/
void	ADC2_DMA_Config(void)
{
	ADC_InitTypeDef			ADC_InitStructure;
	ADC_CommonInitTypeDef	ADC_CommonInitStructure;
	DMA_InitTypeDef			DMA_InitStructure;

/*ADC2 配置*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);										//ADC2挂载在APB2上
	
	ADC_DeInit();
	
	ADC_CommonInitStructure.ADC_DMAAccessMode		=	ADC_DMAAccessMode_Disabled;				//
	ADC_CommonInitStructure.ADC_Mode				=	ADC_Mode_Independent;					//只使用一路ADC，无须同步，所以用独立模式即可
	/*ADC的时钟设为 84MHz/8 =10.5MHz*/
	ADC_CommonInitStructure.ADC_Prescaler			=	ADC_Prescaler_Div4;						//APB2总线时钟频率为84MHz，可设置分频系数2/4/6/8，ADC的时钟需要设置在14MHz以下才可以正常使用??
	/*电流环更新为20KHz，电压值采样需要高于320KHz，每隔32次时钟采一次样*/
	ADC_CommonInitStructure.ADC_TwoSamplingDelay	=	ADC_TwoSamplingDelay_8Cycles;			//相邻两次采样时钟间隔，可设置为5-20，每通道转化时间=采样时间+12个ADC时钟周期=15个时钟周期
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	ADC_InitStructure.ADC_Resolution				=	ADC_Resolution_12b;						//采样分辨率为12位
	ADC_InitStructure.ADC_ScanConvMode				=	ENABLE;									//扫描模式(多通道ADC采集要用扫描模式)
	ADC_InitStructure.ADC_ContinuousConvMode		=	DISABLE;								//关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge		=	ADC_ExternalTrigConvEdge_None;			//没有外部触发,使用软件触发
	ADC_InitStructure.ADC_DataAlign					=	ADC_DataAlign_Right;					//数据格式右对齐
	ADC_InitStructure.ADC_NbrOfConversion			=	2;										//仅使用2个通道
	ADC_Init(ADC2,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC2,ADC_Channel_14,1,ADC_SampleTime_15Cycles);					//规则模式通道设置，力矩传感器1
	ADC_RegularChannelConfig(ADC2,ADC_Channel_15,2,ADC_SampleTime_15Cycles);					//规则模式通道设置，力矩传感器2

/*DMA 配置，使用DMA2_Channel_1 Stream_3*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	DMA_DeInit(DMA2_Stream3);
	DMA_InitStructure.DMA_Channel					=	DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr		=	(uint32_t)(&ADC2->DR);					//使用ADC3的数据寄存器作为DMA的外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr			=	(uint32_t)(&(m_motor_rt_para.m_torque_sensor.torque_adc_buf));//将数组首地址作为内存地址
	DMA_InitStructure.DMA_DIR						=	DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize				=	2;										//buffer size太大，采样时间过长，采样一次很快
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
	DMA_Init(DMA2_Stream3,&DMA_InitStructure);
	DMA_Cmd(DMA2_Stream3,ENABLE);																//使能ADC的DMA
	
/*DMA中断配置*/
	DMA_ClearITPendingBit(DMA2_Stream3,DMA_IT_TC);
	DMA_ITConfig(DMA2_Stream3,DMA_IT_TC,ENABLE);												//设置DMA传输完成中断
	
	ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);											//使能DMA传输完成后请求ADC
	ADC_DMACmd(ADC2,ENABLE);																	//使能ADC的DMA	
	ADC_Cmd(ADC2,ENABLE);																		//使能ADC1
	ADC_SoftwareStartConv(ADC2);																//开启ADC1的软件转换
}




	/*---------------------------------------------------------------------------
	函数名称			：DAC_Config(void)
	参数含义			：null
	函数功能			：使用PA4作为控制电流的输出，调试时使用，PA4原本作为母线电压采集使用
							硬件需要将PA4母线电压采集电路去除
	----------------------------------------------------------------------------*/
void	DAC_Config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	DAC_InitTypeDef		DAC_InitType;
	
	/*开启管脚时钟*/	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);					//使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);						//使能DAC时钟
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	DAC_InitType.DAC_Trigger			=	DAC_Trigger_None;				//不使用触发
	DAC_InitType.DAC_WaveGeneration		=	DAC_WaveGeneration_None;		//不使用波发生器
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude	=	DAC_LFSRUnmask_Bit0;//屏蔽，幅值设置
	DAC_InitType.DAC_OutputBuffer		=	DAC_OutputBuffer_Disable;		//关闭dac1
	DAC_Init(DAC_Channel_1,&DAC_InitType);
	DAC_Cmd(DAC_Channel_1,ENABLE);											//初始化DAC通道1
	
	DAC_Init(DAC_Channel_2,&DAC_InitType);
	DAC_Cmd(DAC_Channel_2,ENABLE);											//初始化DAC通道1
	
	DAC_SetChannel1Data(DAC_Align_12b_R,0);									//设置PA4输出值,0-3300代表
	DAC_SetChannel2Data(DAC_Align_12b_R,0);	
}



	/*CAN总线接口----------------------------------------------------------------
	函数名称			：CAN_Config(void)
	参数含义			：null
	函数功能			：
	----------------------------------------------------------------------------*/
void	CAN_Config(void)
{
	CAN_InitTypeDef			CAN_InitStructure;
	CAN_FilterInitTypeDef	CAN_FilterInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	
/*        CAN总线初始化,CAN总线在APB1总线，频率为42MHz      */
/*			baudrate	=	1/(tq + tbs1 + tbs2)			*/
/*			tq 			=	(BRP + 1) * tpclk				*/
/*			tbs1		=	tq * (TS1 + 1)					*/
/*			tbs2		=	tq * (TS2 + 1)					*/

	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	
	CAN_InitStructure.CAN_ABOM			=	DISABLE;								//DISABLE：软件自动离线管理，ENABLE：如果检测到总线上错误，本节点自动离线
	CAN_InitStructure.CAN_AWUM			=	DISABLE;								//DISABLE：睡眠模式通过软件唤醒，ENABLE：自动唤醒
	CAN_InitStructure.CAN_TTCM			=	DISABLE;								//关闭时间触发通讯模式
	CAN_InitStructure.CAN_NART			=	ENABLE;									//ENABLE：禁止报文自动传送，DISABLE：如果报文发送不成功，自动重发
	CAN_InitStructure.CAN_RFLM			=	DISABLE;								//报文不锁定，在接收报文的FIFO发生溢出，DISABLE：新报文覆盖旧报文，ENABLE：不覆盖旧报文
	CAN_InitStructure.CAN_TXFP			=	DISABLE;								//DISABLE：优先级由报文标识符决定
	CAN_InitStructure.CAN_Mode			=	CAN_Mode_Normal;						//使用普通模式
	CAN_InitStructure.CAN_SJW			=	CAN_SJW_1tq;							//再同步补偿宽度
	CAN_InitStructure.CAN_BS1			=	CAN_BS1_6tq;							//相位缓冲段1
	CAN_InitStructure.CAN_BS2			=	CAN_BS2_7tq;							//相位缓冲段2
	CAN_InitStructure.CAN_Prescaler		=	6;										//时钟预分频系数6
	//baudrate = 1/[(6+7+1)*(6/42M)] = 500K
	
	CAN_Init(CAN1,&CAN_InitStructure);
	
/*CAN滤波器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber			=	0;						//使用滤波器0，共28组，编号0-27
	CAN_FilterInitStructure.CAN_FilterMode				=	CAN_FilterMode_IdMask;	//标识符屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale				=	CAN_FilterScale_32bit;	//使用32位标识符
	CAN_FilterInitStructure.CAN_FilterIdHigh			=	0x0000;					//
	CAN_FilterInitStructure.CAN_FilterIdLow				=	0x0000;					//
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	0x0000;					//
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	0x0000;					//
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	=	CAN_Filter_FIFO0;			//过滤器与FIFO0相关联  CAN_FIFO0;	
	CAN_FilterInitStructure.CAN_FilterActivation		=	ENABLE;					//开启过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0 | CAN_IT_FF0 | CAN_IT_TME,ENABLE);				//FIFO0接收中断；FIFO0满中断；发送邮箱空中断
	
}



/******************************************************************************
*			函数说明：	CAN1_TX_Data(CanTxMsg * msg,uint8_t stdID,uint8_t * data,uint8_t dataLen)
*			参数说明：	CanTxMsg * msg	:要发出的报文结构体
						uint8_t stdID	:标准ID
						uint8_t * data	:数据域数据
						uint8_t dataLen	:数据域长度
*			使用范围：	填写报文内容，使能发送完成中断
*******************************************************************************/
uint8_t CAN1_TX_Data(volatile CanTxMsg * msg,uint32_t stdID,uint8_t * data,uint8_t dataLen)
{
	uint8_t				i;
	
	/*构建CAN Msg报文*/
	msg->StdId			=	stdID;													//标准帧ID
	msg->RTR			=	CAN_RTR_DATA;											//CAN_RTR_DATA:数据帧；CAN_RTR_REMOTE：远程帧
	msg->IDE			=	CAN_Id_Standard;										//CAN_Id_Standard：发送标准帧格式；CAN_Id_Extended：发送扩展帧格式
	msg->ExtId			=	0x12;													//扩展帧ID
	msg->DLC			=	dataLen;												//数据域长度，小于8
	
	/*数据域赋值时，进入临界区*/
	OSIntEnter();
	for(i=0;i<dataLen;i++)
	{
		msg->Data[i]	=	data[i];												//数据域赋值
	}
	OSIntExit();

	/*发送报文*/

	m_can.mbox		=	CAN_Transmit(CAN1,(CanTxMsg*)msg);							//CAN有三个发送邮箱，发送函数会返回邮箱序号，即mbox值
	if(CAN_NO_MB == m_can.mbox)
	{
		return	0;																	//如果没有可用的发送邮箱，返回0
	}
	else
	{
		m_can.CAN_msg_num[m_can.mbox]	=	1;
	}
	
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
	return	1;
}



	/*---------------------------------------------------------------------------
	函数名称			：BrakeControl(void)
	参数含义			：mode:		1 开启抱闸		0 关闭抱闸		其它 无操作
	函数功能			：开启关闭抱闸使用函数
	----------------------------------------------------------------------------*/
void	BrakeControl(uint8_t	mode)
{
//	uint8_t 	cnt = 0;
//	uint32_t	io_state;
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
	函数名称			：LED_GREEN_ON(void)
	参数含义			：
	函数功能			：绿色LED点亮
	----------------------------------------------------------------------------*/
void	LED_GREEN_ON(void)
{
	GPIOC->ODR |= 0x0002;
}

	/*---------------------------------------------------------------------------
	函数名称			：LED_GREEN_OFF(void)
	参数含义			：
	函数功能			：绿色LED熄灭
	----------------------------------------------------------------------------*/
void	LED_GREEN_OFF(void)
{
	GPIOC->ODR &= 0xFFFD;
}


	/*---------------------------------------------------------------------------
	函数名称			：LED_RED_ON(void)
	参数含义			：
	函数功能			：绿色LED点亮
	----------------------------------------------------------------------------*/
void	LED_RED_ON(void)
{
	GPIOC->ODR |= 0x0004;
}

	/*---------------------------------------------------------------------------
	函数名称			：LED_RED_OFF(void)
	参数含义			：
	函数功能			：绿色LED熄灭
	----------------------------------------------------------------------------*/
void	LED_RED_OFF(void)
{
	GPIOC->ODR &= 0xFFFB;
}

/**********************end of file************************/
