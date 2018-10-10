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
	��������			��NVIC_Config(void)
	��������			��null
	��������			���ж����������ȼ�����
							�ж�������ѡ��2
							����4����ռʽ�ж����ȼ�		0-3
							�Լ�4����Ӧ���ȼ�			0-3
	
							TIM1_UP			pwm��������ж�	20KHz
							TIM2			��������ʱ�ж�  20KHz
							TIM3			���������������������ж�	
							TIM4			���������������ж�
							TIM8			�ٶȻ�λ�û�����
							DMA2_Stream0	ADC��������ж�
	----------------------------------------------------------------------------*/
void	NVIC_Config(void)
{
	NVIC_InitTypeDef 	NVIC_InitStructure;
	
	/*PWM�����������������Լ���ʱ����ADC������TIM1 ��������ж�*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/*Hall ���������벶���жϣ�TIM4���벶���ж�*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);	
	
	/*ADC-DMA ��������жϣ���������*/
	NVIC_InitStructure.NVIC_IRQChannel						=	DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	2;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/*�ٶȻ���λ�û������жϣ�TIM2 ��������ж�*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	3;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);	

	/*TIM8 �ٶ�λ�û�����ʱ��Դ*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM8_UP_TIM13_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	/*SPI 1 DMA RX�ж�*/
	NVIC_InitStructure.NVIC_IRQChannel						=	DMA2_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;						//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1;						//��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);	
		
	/*TIM3 CH3�����ж�*/
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;						//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	2;						//��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);			
		
	/*ADC-DMA ��������жϣ����ش��������*/
	NVIC_InitStructure.NVIC_IRQChannel						=	DMA2_Stream3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);	
	
	/*CAN BUS ���շ����жϳ�ʼ��*/
	NVIC_InitStructure.NVIC_IRQChannel						=	CAN1_RX0_IRQn;			//FIFO0�Ľ����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;						//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1;						//
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;					//
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel						=	CAN1_TX_IRQn;			//FIFO0�ķ����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;						//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	2;						//
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;					//
	NVIC_Init(&NVIC_InitStructure);	


	
	

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

/*LED�ܽ����� PC1/2*/	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_2MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_1|GPIO_Pin_2);
	
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

/*����ʽ������T�����ٶ˿�*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_0;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);

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
	
/*ADģ�����ɼ�����������ɼ�ʹ�� PA1/2/3,20180914*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	
/*ADģ�����ɼ������ش������ɼ�ʹ�� PC4/5*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
	
/*ADģ�����ɼ���ĸ�ߵ�ѹ�ɼ�ʹ�� PC3*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_3;
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
	
/*CAN���߽ӿ�*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType		=	GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
	
/*SPI 1 �˿ڳ�ʼ�� ��ȡ����ֵ����������***********/
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
	��������			��	SPI1ʹ��DMA��ʽͨѶ
							SPI1	-	CLK		-	PB3
							SPI1	-	MISO	-	PB4

							SPI1 TX	-	DMA2 Stream3 Ch3
							SPI1 RX	-	DMA2 Stream2 Ch3
	----------------------------------------------------------------------------*/
void	SPI1_DMA_Config(void)
{
	SPI_InitTypeDef		SPI_InitStructure;
	DMA_InitTypeDef		DMA_InitStructure;
	
/*ʱ������************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
/*SPI 1 ����************/
	SPI_I2S_DeInit(SPI1);
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_BaudRatePrescaler	=	SPI_BaudRatePrescaler_256;							//����ʱ��84MHz
	SPI_InitStructure.SPI_Direction			=	SPI_Direction_2Lines_FullDuplex;//SPI_Direction_1Line_Rx;//
	SPI_InitStructure.SPI_Mode				=	SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize			=	SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPHA				=	SPI_CPHA_2Edge;										//�ڶ��������ض�ȡ����
	SPI_InitStructure.SPI_CPOL				=	SPI_CPOL_High;										//���߿���Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_NSS				=	SPI_NSS_Soft;										//�������Ƭѡ�ź�
	SPI_InitStructure.SPI_FirstBit			=	SPI_FirstBit_MSB;									//��λ��ǰ
	SPI_Init(SPI1,&SPI_InitStructure);
	SPI_CalculateCRC(SPI1,DISABLE);																	//�ر�CRCУ��
	SPI_Cmd(SPI1,ENABLE);																			//ʹ��SPI1
	
/*SPI DMA Init*/
	DMA_DeInit(DMA2_Stream2);
	DMA_DeInit(DMA2_Stream3);
	
/*Rx SPI1 DMAͨ����ʼ��*/
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel				=	DMA_Channel_3;										//DMAͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr	=	(uint32_t)(&SPI1->DR);								//DMA�����ַΪSPI�����ݼĴ���
	DMA_InitStructure.DMA_Memory0BaseAddr		=	(uint32_t)(&m_motor_rt_para.m_abs_encoder.u8_abs_raw_data[0]);//DMA���ڴ��ַ����
	DMA_InitStructure.DMA_DIR					=	DMA_DIR_PeripheralToMemory;							//DMA����Ϊ������Ĵ������ڴ�
	DMA_InitStructure.DMA_BufferSize			=	4;													//DMA��Buffer���ô�С��ʵ��ʹ��ʱ�����Ķ�
	DMA_InitStructure.DMA_PeripheralInc			=	DMA_PeripheralInc_Disable;							//�����ַ����
	DMA_InitStructure.DMA_MemoryInc				=	DMA_MemoryInc_Enable;								//�ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;						//�������ݿ��Ϊ8bits
	DMA_InitStructure.DMA_MemoryDataSize		=	DMA_MemoryDataSize_Byte;							//�ڴ����ݿ�Ϊ8bits
	DMA_InitStructure.DMA_Mode					=	DMA_Mode_Normal;									//
	DMA_InitStructure.DMA_Priority				=	DMA_Priority_High;									//DMA���ȼ���
	DMA_InitStructure.DMA_FIFOMode				=	DMA_FIFOMode_Disable;								//����fifoģʽ��ʹ��ֱ��ģʽ
	DMA_InitStructure.DMA_FIFOThreshold			=	DMA_FIFOThreshold_Full;								//fifo����ֵ
	DMA_InitStructure.DMA_MemoryBurst			=	DMA_MemoryBurst_Single;								//�ڴ浥�δ������ݿ�ȣ�ÿ��ת��һ������
	DMA_InitStructure.DMA_PeripheralBurst		=	DMA_PeripheralBurst_Single;							//���赥�δ������ݿ�ȣ�ÿ��ת��һ������
	DMA_Init(DMA2_Stream2,&DMA_InitStructure);
	
/*Tx SPI1 DMAͨ����ʼ��*/
	DMA_InitStructure.DMA_Channel				=	DMA_Channel_3;
	DMA_InitStructure.DMA_PeripheralBaseAddr	=	(uint32_t)(&SPI1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr		=	(uint32_t)(&wdata_temp[0]);
	DMA_InitStructure.DMA_DIR					=	DMA_DIR_MemoryToPeripheral;	
	DMA_InitStructure.DMA_BufferSize			=	4;
	DMA_InitStructure.DMA_Priority				=	DMA_Priority_Low;
	DMA_Init(DMA2_Stream3,&DMA_InitStructure);
	
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,ENABLE);
	
/*SPI DMA Rx�ж�����*/	
	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC | DMA_IT_TE, ENABLE);
/*SPI DMA Tx�ж�����*/
	DMA_ITConfig(DMA2_Stream3, DMA_IT_TC | DMA_IT_TE, ENABLE);

	DMA_Cmd(DMA2_Stream2,DISABLE);
	DMA_Cmd(DMA2_Stream3,DISABLE);	
	DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);
	DMA_ClearITPendingBit(DMA2_Stream3,DMA_IT_TCIF3);
}


	/*---------------------------------------------------------------------------
	��������			��	SPI_DMA_ReadData
	��������			��	length			��ȡ���ݳ���
	��������			��	ͨ��SPI DMA��ʽ��ȡ���Ա�����λ��
							��ȡ���ݴ洢��readBuf[1]��ʼ�ĵط���readBuf[0]�в���
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

	/*PWM����������������--------------------------------------------------------
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
	TIM_BaseInitStructure.TIM_Period				=	PWM_CYCLE_U16;							//����ֵ�������
	TIM_BaseInitStructure.TIM_Prescaler				=	0;										//Ԥ��Ƶ��Ϊ1
	TIM_BaseInitStructure.TIM_RepetitionCounter		=	0;
	TIM_TimeBaseInit(TIM1,&TIM_BaseInitStructure);
	
/*��ʼ����ʱ���ĸ�ͨ��*/
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode					=	TIM_OCMode_PWM2;						//pwmģʽ1��������ϼ���ʱ��cnt<ccrʱΪ��Ч��ƽ�����¼���ʱ��cnt>ccrʱΪ��Ч��ƽ																								//pwmģʽ2��������ϼ���ʱ��cnt<ccrʱΪ��Ч��ƽ�����¼���ʱ��cnt>ccrʱΪ��Ч��ƽ
	if(m_motor_ctrl.m_sys_state.u8_use_svpwm		==	USE_FOC)
	{
		TIM_OCInitStructure.TIM_OCIdleState				=	TIM_OCIdleState_Set;				//������pulseֵ�����͹ܽţ�����ʱ�����
		TIM_OCInitStructure.TIM_OCNIdleState			=	TIM_OCNIdleState_Set;
		TIM_OCInitStructure.TIM_OCPolarity				=	TIM_OCPolarity_High;				//��Ч��ƽΪ��
		TIM_OCInitStructure.TIM_OCNPolarity				=	TIM_OCNPolarity_High;				//�������ͨ������Ч��ƽΪ�ͣ���TIM_OCPolarity����һ�²��ܻ���
	}
	else
	{
		TIM_OCInitStructure.TIM_OCIdleState				=	TIM_OCIdleState_Reset;				//������pulseֵ�����͹ܽţ�����ʱ�����
		TIM_OCInitStructure.TIM_OCNIdleState			=	TIM_OCNIdleState_Reset;
		TIM_OCInitStructure.TIM_OCPolarity				=	TIM_OCPolarity_Low;					//��Ч��ƽΪ��
		TIM_OCInitStructure.TIM_OCNPolarity				=	TIM_OCNPolarity_Low;				//�������ͨ������Ч��ƽΪ�ͣ���TIM_OCPolarity����һ�²��ܻ���	
	}
	TIM_OCInitStructure.TIM_OutputState				=	TIM_OutputState_Enable;					//���ͨ��ʹ��
	TIM_OCInitStructure.TIM_OutputNState			=	TIM_OutputNState_Enable;				//�������ͨ�������ʹ��
	TIM_OCInitStructure.TIM_Pulse					=	10;										//ռ�ձ�����
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
	TIM_BDTRInitStructure.TIM_DeadTime				=	0x54;//0xE0;							//����ʱ�䣬�м����ģʽ�£�ǰ�����������ʱ����N/84 us,IR2130S������ʱ��Ϊ2us��������㣬�μ�BDTR�Ĵ�������
	TIM_BDTRInitStructure.TIM_Break					=	TIM_Break_Disable;						//ɲ������ʹ��
	TIM_BDTRInitStructure.TIM_BreakPolarity			=	TIM_BreakPolarity_High;					//ɲ������
	TIM_BDTRInitStructure.TIM_AutomaticOutput		=	TIM_AutomaticOutput_Enable;				//�Զ����ʹ��
	TIM_BDTRConfig(TIM1,&TIM_BDTRInitStructure);

/*ǿ��H1-3,L1-3�����*/
	IR2130S_force_reset();
	TIM_ARRPreloadConfig(TIM1,ENABLE);															//ʹ���Զ���װ�ؼĴ���
	TIM_Cmd(TIM1,ENABLE);																		//���ö�ʱ��1
	TIM_CtrlPWMOutputs(TIM1,ENABLE);															//ʹ��timer1���PWM
}


	/*�ٶ�λ�û�����-------------------------------------------------------------
	��������			��Timer2_Config(void)
	��������			��null
	��������			��	����1KHzƵ���жϣ������ٶ�λ�û���PI����
							TIM2ʱ����Ϊ�Ӷ�ʱ��ʹ�ã�������ʱ��ΪTIM8��
							TIM8����10KHzʱ���źţ�
							Ҫ����λ�û��ٶȻ�����Ҫ��TIM2
	----------------------------------------------------------------------------*/
void	Timer2_Config(void)
{
	TIM_TimeBaseInitTypeDef 		TIM_TimeBaseInitStructure;

/*ʱ������*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
/*Timer2����*/
	TIM_DeInit(TIM2);																			//Timer 2 ��APB1�ϣ�42MHz����system_stm32f4xx.c��
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_ClockDivision				=	TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode				=	TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period					=	5-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler					=	2-1;			
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_SelectSlaveMode(TIM2,TIM_SlaveMode_Trigger);											//TIM2ѡ���ģʽ�Ĵ���ģʽ
	TIM_SelectInputTrigger(TIM2,TIM_TS_ITR1);													//ѡ��TIM8������ź���Ϊʱ��Դ
	TIM_ITRxExternalClockConfig(TIM2,TIM_TS_ITR1);												//ѡ���ⲿʱ�Ӵ���

	TIM_Cmd(TIM2,ENABLE);
	TIM_ClearFlag(TIM2,TIM_IT_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
}


	/*�������������-------------------------------------------------------------
	��������			��Timer3_Config(void)
	��������			��null
	��������			�����ڽ��չ����������ź�			timer3 APB1��42MHz
							A��		TIM3->Channel1
							B��		TIM3->Channel2
							C��		TIM3 ���벶��
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
	TIM_TimeBaseInitStructure.TIM_Period				=	65535;								//�����嵽���ֵʱ������װ�أ�������һ���ж�
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
	
/*����ͨ������*/
	TIM_ICInitStructure.TIM_Channel						=	TIM_Channel_3;						//������I������
	TIM_ICInitStructure.TIM_ICPolarity					=	TIM_ICPolarity_Rising;				//ֻ����������
	TIM_ICInitStructure.TIM_ICSelection					=	TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICFilter					=	0;
	TIM_ICInitStructure.TIM_ICPrescaler					=	0;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_CC3,ENABLE);//TIM_IT_Update|TIM_IT_CC3
	TIM3->CNT = 0;
	TIM_Cmd(TIM3,ENABLE);	
}


	/*�����ж��ź�---------------------------------------------------------------
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
	TIM_ClearFlag(TIM4,TIM_IT_CC1);
	TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);													//ʹ�ܸ����ж�
	TIM_Cmd(TIM4,ENABLE);																	//������ʱ��4
}




	/*����T������----------------------------------------------------------------
	��������			��	Timer5_Config(void)
	��������			��	null
	��������			��	���ڸ��ٶ˹�����������ֵ����������������
							TIM5 CH1���������PWM����ģʽ
							TIM5 ����APB1�ϣ�����ʱ��Ƶ��42MHz��������

							��¼����������ֵͨ��DMA���͸���Ӧ�ڴ��ַ��
							ʹ��DMAͨ����DMA1 stream 1 channel 6



							���ٲ����������  1.56ms
	----------------------------------------------------------------------------*/
void	Timer5_Config(void)
{
	TIM_TimeBaseInitTypeDef 		TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef				TIM_ICInitStructure;	
	DMA_InitTypeDef					DMA_InitStructure;
	
/*ʱ��ʹ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE );

/*dma����*/
	DMA_DeInit(DMA1_Stream2);
	DMA_InitStructure.DMA_Channel					=	DMA_Channel_6;
	DMA_InitStructure.DMA_PeripheralBaseAddr		=	(uint32_t)(&TIM5->CCR1);				//ʹ��TIM5 CCR1�����ݼĴ�����ΪDMA�������ַ
	DMA_InitStructure.DMA_Memory0BaseAddr			=	(uint32_t)(&(m_motor_rt_para.m_inc_encoder.u32_pulse_width_buf));//�������׵�ַ��Ϊ�ڴ��ַ
	DMA_InitStructure.DMA_DIR						=	DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize				=	PW_BUFFER_LENGTH;						//
	DMA_InitStructure.DMA_PeripheralInc				=	DMA_PeripheralInc_Disable;				//DMA��������������ַ������
	DMA_InitStructure.DMA_MemoryInc					=	DMA_MemoryInc_Enable;					//DMA����������ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize		=	DMA_PeripheralDataSize_Word;			//ʹ��16λ������Ϊ���䵥λ������TIM5->CCR1Ϊ32λ�Ĵ���������ARRΪ16λ��
	DMA_InitStructure.DMA_MemoryDataSize			=	DMA_MemoryDataSize_Word;				//
	DMA_InitStructure.DMA_Mode						=	DMA_Mode_Circular;						//����DMA����ģʽΪѭ������
	DMA_InitStructure.DMA_Priority					=	DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode					=	DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold				=	DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst				=	DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst			=	DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream2,&DMA_InitStructure);
	DMA_Cmd(DMA1_Stream2,ENABLE);																//ʹ��ADC��DMA
	
	
/*Timer5 ����*/
	TIM_DeInit(TIM5);
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_CounterMode			=	TIM_CounterMode_Up;					//���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision			=	TIM_CKD_DIV1;						//TIM5ʱ�ӷָ
	TIM_TimeBaseInitStructure.TIM_Period				=	0xFFFFFFFF;							//�����嵽���ֵʱ������װ�أ�������һ���жϣ���ζ�Ż�ȡ������������65535��ʱ����������
	TIM_TimeBaseInitStructure.TIM_Prescaler				=	1;									//2��Ԥ��Ƶ��ʹ��42MHzƵ��
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
/*���ö�ʱ�����벶��ͨ��*/	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel						=	TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity					=	TIM_ICPolarity_Rising;				//��������ĵ�ƽ���½��ز���
	TIM_ICInitStructure.TIM_ICSelection					=	TIM_ICSelection_DirectTI;			//ӳ�䵽TI1��
	TIM_ICInitStructure.TIM_ICPrescaler					=	TIM_ICPSC_DIV1;						//���벶���Ƶ���˴���ÿ�����嶼��׽
	TIM_ICInitStructure.TIM_ICFilter					=	0;									//�����˲�����ÿ�����嶼��Ч
	TIM_PWMIConfig(TIM5,&TIM_ICInitStructure);
	
	TIM_SelectInputTrigger(TIM5,TIM_TS_TI1FP1);													//��������ѡ��ʹ��IN1�ź���Ϊ����
	TIM_SelectSlaveMode(TIM5,TIM_SlaveMode_Reset);												//TIM�ӻ�ģʽ�������źŵ����������³�ʼ���������ʹ����Ĵ����ĸ����¼�
	TIM_SelectMasterSlaveMode(TIM5,TIM_MasterSlaveMode_Enable);									//������ʱ��������������
	TIM_DMAConfig(TIM5,TIM_DMABase_CCR1,TIM_DMABurstLength_2Bytes);								//���ö�ʱ����DMA��ÿ�ζ��Ĵ�����2bytes
	TIM_DMACmd(TIM5,TIM_DMA_CC1,ENABLE);														//ʹ��CCR1 dma����
	
	TIM5->CNT = 0;
	TIM_Cmd(TIM5,ENABLE);	
}

	/*�������ٶȻ�ͬ����ʱ����������ֵ������-------------------------------------------------------
	��������			��	Timer8_Config(void)
	��������			��	null
	��������			��	���� ��ʱ����spi dma��
							TIM8 ����APB2�ϣ�ʱ��Ϊ168Mhz
							��������ģʽ��10KHz����TIM2���ٶ�λ�û����£���Ϊ�Ӷ�ʱ��
							TIM8�����ź�ΪITR0
							�ڶ�ʱ���ж��п���spi dma��ȡ�źţ���ȡssi�ź�ֵ��ʱ����
							200us����
	----------------------------------------------------------------------------*/
void	Timer8_Master_Config(void)
{
	TIM_TimeBaseInitTypeDef	TIM_BaseInitStructure;
	TIM_ICInitTypeDef		TIM_ICInitStructure;

/*ʱ�ӿ���*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	
/*��ʱ����������*/
	TIM_BaseInitStructure.TIM_ClockDivision			=	TIM_CKD_DIV1;						//ʱ�ӷ�Ƶϵ��1��ʹ��168Mhzʱ��Ƶ��
	TIM_BaseInitStructure.TIM_CounterMode			=	TIM_CounterMode_Up;					//���ϼ���ģʽ
	TIM_BaseInitStructure.TIM_Period				=	840 - 1;							//�������ֵ
	TIM_BaseInitStructure.TIM_Prescaler				=	20 - 1;								//Ԥ��Ƶϵ��2��ʹ��84MHz
	TIM_TimeBaseInit(TIM8,&TIM_BaseInitStructure);

	TIM_SelectOutputTrigger(TIM8,TIM_TRGOSource_Update);									//ѡ��TIM9�ļ�������ź�ΪTRGO
	TIM_SelectMasterSlaveMode(TIM8,TIM_MasterSlaveMode_Enable);								//��TIM9�ļ���ģʽ
	
/*TIM9�ж����ã���ʱ������ж�*/
	TIM_ClearFlag(TIM8,TIM_IT_Update);
	TIM_ITConfig(TIM8,TIM_IT_Update,ENABLE);												//ʹ�ܸ����ж�
	TIM_Cmd(TIM8,ENABLE);
}



	/*���������DMA2��ʽ----------------------------------------------------------
	��������			��ADC1_DMA_Config(void)
	��������			��null
	��������			��ADC������DMA��ʼ��
							PA1				ADC123_IN_1			U�����
							PA2				ADC123_IN_2			V�����
							PA3				ADC123_IN_3			W�����
							ADC1	DMA2  stream 0 channel 0
	----------------------------------------------------------------------------*/
void	ADC1_DMA_Config(void)
{
	ADC_InitTypeDef			ADC_InitStructure;
	ADC_CommonInitTypeDef	ADC_CommonInitStructure;
	DMA_InitTypeDef			DMA_InitStructure;

/*ADC1 ����*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);										//ADC1������APB2��
	
	ADC_DeInit();
	
	ADC_CommonInitStructure.ADC_DMAAccessMode		=	ADC_DMAAccessMode_Disabled;				//
	ADC_CommonInitStructure.ADC_Mode				=	ADC_Mode_Independent;					//ֻʹ��һ·ADC������ͬ���������ö���ģʽ����
	/*ADC��ʱ����Ϊ 84MHz/8 =10.5MHz*/
	ADC_CommonInitStructure.ADC_Prescaler			=	ADC_Prescaler_Div4;						//APB2����ʱ��Ƶ��Ϊ84MHz�������÷�Ƶϵ��2/4/6/8��ADC��ʱ����Ҫ������14MHz���²ſ�������ʹ��??
	/*����������Ϊ20KHz����ѹֵ������Ҫ����320KHz��ÿ��32��ʱ�Ӳ�һ����*/
	ADC_CommonInitStructure.ADC_TwoSamplingDelay	=	ADC_TwoSamplingDelay_8Cycles;			//�������β���ʱ�Ӽ����������Ϊ5-20��ÿͨ��ת��ʱ��=����ʱ��+12��ADCʱ������=15��ʱ������
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	ADC_InitStructure.ADC_Resolution				=	ADC_Resolution_12b;						//�����ֱ���Ϊ12λ
	ADC_InitStructure.ADC_ScanConvMode				=	ENABLE;									//ɨ��ģʽ(��ͨ��ADC�ɼ�Ҫ��ɨ��ģʽ)
	ADC_InitStructure.ADC_ContinuousConvMode		=	DISABLE;								//�ر�����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge		=	ADC_ExternalTrigConvEdge_None;			//û���ⲿ����,ʹ���������
	ADC_InitStructure.ADC_DataAlign					=	ADC_DataAlign_Right;					//���ݸ�ʽ�Ҷ���
	ADC_InitStructure.ADC_NbrOfConversion			=	3;										//��ʹ��3��ͨ��
	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_15Cycles);						//����ģʽͨ�����ã�U
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,2,ADC_SampleTime_15Cycles);						//����ģʽͨ�����ã�V
	ADC_RegularChannelConfig(ADC1,ADC_Channel_3,3,ADC_SampleTime_15Cycles);						//����ģʽͨ�����ã�W

/*DMA ���ã�ʹ��DMA2_Channel_0 Stream_0*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel					=	DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr		=	(uint32_t)(&ADC1->DR);					//ʹ��ADC3�����ݼĴ�����ΪDMA�������ַ
	DMA_InitStructure.DMA_Memory0BaseAddr			=	(uint32_t)(&(m_motor_rt_para.m_current_sensor.ADC_DMA_buf));//�������׵�ַ��Ϊ�ڴ��ַ
	DMA_InitStructure.DMA_DIR						=	DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize				=	3;										//buffer size̫�󣬲���ʱ�����������һ�κܿ�
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
	
/*DMA�ж�����*/
	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TC);
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);												//����DMA��������ж�
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);											//ʹ��DMA������ɺ�����ADC
	ADC_DMACmd(ADC1,ENABLE);																	//ʹ��ADC��DMA	
	ADC_Cmd(ADC1,ENABLE);																		//ʹ��ADC1
	ADC_SoftwareStartConv(ADC1);																//����ADC1�����ת��
}



	/*���ش��������DMA2��ʽ----------------------------------------------------------
	��������			��ADC2_DMA_Config(void)
	��������			��null
	��������			��ADC������DMA��ʼ��
							PC4				ADC12_IN_14			T_sen_1
							PC5				ADC12_IN_15			T_sen_2							
							ADC2	DMA2  stream 3 channel 1
	----------------------------------------------------------------------------*/
void	ADC2_DMA_Config(void)
{
	ADC_InitTypeDef			ADC_InitStructure;
	ADC_CommonInitTypeDef	ADC_CommonInitStructure;
	DMA_InitTypeDef			DMA_InitStructure;

/*ADC2 ����*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);										//ADC2������APB2��
	
	ADC_DeInit();
	
	ADC_CommonInitStructure.ADC_DMAAccessMode		=	ADC_DMAAccessMode_Disabled;				//
	ADC_CommonInitStructure.ADC_Mode				=	ADC_Mode_Independent;					//ֻʹ��һ·ADC������ͬ���������ö���ģʽ����
	/*ADC��ʱ����Ϊ 84MHz/8 =10.5MHz*/
	ADC_CommonInitStructure.ADC_Prescaler			=	ADC_Prescaler_Div4;						//APB2����ʱ��Ƶ��Ϊ84MHz�������÷�Ƶϵ��2/4/6/8��ADC��ʱ����Ҫ������14MHz���²ſ�������ʹ��??
	/*����������Ϊ20KHz����ѹֵ������Ҫ����320KHz��ÿ��32��ʱ�Ӳ�һ����*/
	ADC_CommonInitStructure.ADC_TwoSamplingDelay	=	ADC_TwoSamplingDelay_8Cycles;			//�������β���ʱ�Ӽ����������Ϊ5-20��ÿͨ��ת��ʱ��=����ʱ��+12��ADCʱ������=15��ʱ������
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	ADC_InitStructure.ADC_Resolution				=	ADC_Resolution_12b;						//�����ֱ���Ϊ12λ
	ADC_InitStructure.ADC_ScanConvMode				=	ENABLE;									//ɨ��ģʽ(��ͨ��ADC�ɼ�Ҫ��ɨ��ģʽ)
	ADC_InitStructure.ADC_ContinuousConvMode		=	DISABLE;								//�ر�����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge		=	ADC_ExternalTrigConvEdge_None;			//û���ⲿ����,ʹ���������
	ADC_InitStructure.ADC_DataAlign					=	ADC_DataAlign_Right;					//���ݸ�ʽ�Ҷ���
	ADC_InitStructure.ADC_NbrOfConversion			=	2;										//��ʹ��2��ͨ��
	ADC_Init(ADC2,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC2,ADC_Channel_14,1,ADC_SampleTime_15Cycles);					//����ģʽͨ�����ã����ش�����1
	ADC_RegularChannelConfig(ADC2,ADC_Channel_15,2,ADC_SampleTime_15Cycles);					//����ģʽͨ�����ã����ش�����2

/*DMA ���ã�ʹ��DMA2_Channel_1 Stream_3*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	DMA_DeInit(DMA2_Stream3);
	DMA_InitStructure.DMA_Channel					=	DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr		=	(uint32_t)(&ADC2->DR);					//ʹ��ADC3�����ݼĴ�����ΪDMA�������ַ
	DMA_InitStructure.DMA_Memory0BaseAddr			=	(uint32_t)(&(m_motor_rt_para.m_torque_sensor.torque_adc_buf));//�������׵�ַ��Ϊ�ڴ��ַ
	DMA_InitStructure.DMA_DIR						=	DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize				=	2;										//buffer size̫�󣬲���ʱ�����������һ�κܿ�
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
	DMA_Init(DMA2_Stream3,&DMA_InitStructure);
	DMA_Cmd(DMA2_Stream3,ENABLE);																//ʹ��ADC��DMA
	
/*DMA�ж�����*/
	DMA_ClearITPendingBit(DMA2_Stream3,DMA_IT_TC);
	DMA_ITConfig(DMA2_Stream3,DMA_IT_TC,ENABLE);												//����DMA��������ж�
	
	ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);											//ʹ��DMA������ɺ�����ADC
	ADC_DMACmd(ADC2,ENABLE);																	//ʹ��ADC��DMA	
	ADC_Cmd(ADC2,ENABLE);																		//ʹ��ADC1
	ADC_SoftwareStartConv(ADC2);																//����ADC1�����ת��
}




	/*---------------------------------------------------------------------------
	��������			��DAC_Config(void)
	��������			��null
	��������			��ʹ��PA4��Ϊ���Ƶ��������������ʱʹ�ã�PA4ԭ����Ϊĸ�ߵ�ѹ�ɼ�ʹ��
							Ӳ����Ҫ��PA4ĸ�ߵ�ѹ�ɼ���·ȥ��
	----------------------------------------------------------------------------*/
void	DAC_Config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	DAC_InitTypeDef		DAC_InitType;
	
	/*�����ܽ�ʱ��*/	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);					//ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);						//ʹ��DACʱ��
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin			=	GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_PuPd		=	GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	DAC_InitType.DAC_Trigger			=	DAC_Trigger_None;				//��ʹ�ô���
	DAC_InitType.DAC_WaveGeneration		=	DAC_WaveGeneration_None;		//��ʹ�ò�������
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude	=	DAC_LFSRUnmask_Bit0;//���Σ���ֵ����
	DAC_InitType.DAC_OutputBuffer		=	DAC_OutputBuffer_Disable;		//�ر�dac1
	DAC_Init(DAC_Channel_1,&DAC_InitType);
	DAC_Cmd(DAC_Channel_1,ENABLE);											//��ʼ��DACͨ��1
	
	DAC_Init(DAC_Channel_2,&DAC_InitType);
	DAC_Cmd(DAC_Channel_2,ENABLE);											//��ʼ��DACͨ��1
	
	DAC_SetChannel1Data(DAC_Align_12b_R,0);									//����PA4���ֵ,0-3300����
	DAC_SetChannel2Data(DAC_Align_12b_R,0);	
}



	/*CAN���߽ӿ�----------------------------------------------------------------
	��������			��CAN_Config(void)
	��������			��null
	��������			��
	----------------------------------------------------------------------------*/
void	CAN_Config(void)
{
	CAN_InitTypeDef			CAN_InitStructure;
	CAN_FilterInitTypeDef	CAN_FilterInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	
/*        CAN���߳�ʼ��,CAN������APB1���ߣ�Ƶ��Ϊ42MHz      */
/*			baudrate	=	1/(tq + tbs1 + tbs2)			*/
/*			tq 			=	(BRP + 1) * tpclk				*/
/*			tbs1		=	tq * (TS1 + 1)					*/
/*			tbs2		=	tq * (TS2 + 1)					*/

	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	
	CAN_InitStructure.CAN_ABOM			=	DISABLE;								//DISABLE������Զ����߹���ENABLE�������⵽�����ϴ��󣬱��ڵ��Զ�����
	CAN_InitStructure.CAN_AWUM			=	DISABLE;								//DISABLE��˯��ģʽͨ��������ѣ�ENABLE���Զ�����
	CAN_InitStructure.CAN_TTCM			=	DISABLE;								//�ر�ʱ�䴥��ͨѶģʽ
	CAN_InitStructure.CAN_NART			=	ENABLE;									//ENABLE����ֹ�����Զ����ͣ�DISABLE��������ķ��Ͳ��ɹ����Զ��ط�
	CAN_InitStructure.CAN_RFLM			=	DISABLE;								//���Ĳ��������ڽ��ձ��ĵ�FIFO���������DISABLE���±��ĸ��Ǿɱ��ģ�ENABLE�������Ǿɱ���
	CAN_InitStructure.CAN_TXFP			=	DISABLE;								//DISABLE�����ȼ��ɱ��ı�ʶ������
	CAN_InitStructure.CAN_Mode			=	CAN_Mode_Normal;						//ʹ����ͨģʽ
	CAN_InitStructure.CAN_SJW			=	CAN_SJW_1tq;							//��ͬ���������
	CAN_InitStructure.CAN_BS1			=	CAN_BS1_6tq;							//��λ�����1
	CAN_InitStructure.CAN_BS2			=	CAN_BS2_7tq;							//��λ�����2
	CAN_InitStructure.CAN_Prescaler		=	6;										//ʱ��Ԥ��Ƶϵ��6
	//baudrate = 1/[(6+7+1)*(6/42M)] = 500K
	
	CAN_Init(CAN1,&CAN_InitStructure);
	
/*CAN�˲�����ʼ��*/
	CAN_FilterInitStructure.CAN_FilterNumber			=	0;						//ʹ���˲���0����28�飬���0-27
	CAN_FilterInitStructure.CAN_FilterMode				=	CAN_FilterMode_IdMask;	//��ʶ������λģʽ
	CAN_FilterInitStructure.CAN_FilterScale				=	CAN_FilterScale_32bit;	//ʹ��32λ��ʶ��
	CAN_FilterInitStructure.CAN_FilterIdHigh			=	0x0000;					//
	CAN_FilterInitStructure.CAN_FilterIdLow				=	0x0000;					//
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	0x0000;					//
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	0x0000;					//
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	=	CAN_Filter_FIFO0;			//��������FIFO0�����  CAN_FIFO0;	
	CAN_FilterInitStructure.CAN_FilterActivation		=	ENABLE;					//����������
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0 | CAN_IT_FF0 | CAN_IT_TME,ENABLE);				//FIFO0�����жϣ�FIFO0���жϣ�����������ж�
	
}



/******************************************************************************
*			����˵����	CAN1_TX_Data(CanTxMsg * msg,uint8_t stdID,uint8_t * data,uint8_t dataLen)
*			����˵����	CanTxMsg * msg	:Ҫ�����ı��Ľṹ��
						uint8_t stdID	:��׼ID
						uint8_t * data	:����������
						uint8_t dataLen	:�����򳤶�
*			ʹ�÷�Χ��	��д�������ݣ�ʹ�ܷ�������ж�
*******************************************************************************/
uint8_t CAN1_TX_Data(volatile CanTxMsg * msg,uint32_t stdID,uint8_t * data,uint8_t dataLen)
{
	uint8_t				i;
	
	/*����CAN Msg����*/
	msg->StdId			=	stdID;													//��׼֡ID
	msg->RTR			=	CAN_RTR_DATA;											//CAN_RTR_DATA:����֡��CAN_RTR_REMOTE��Զ��֡
	msg->IDE			=	CAN_Id_Standard;										//CAN_Id_Standard�����ͱ�׼֡��ʽ��CAN_Id_Extended��������չ֡��ʽ
	msg->ExtId			=	0x12;													//��չ֡ID
	msg->DLC			=	dataLen;												//�����򳤶ȣ�С��8
	
	/*������ֵʱ�������ٽ���*/
	OSIntEnter();
	for(i=0;i<dataLen;i++)
	{
		msg->Data[i]	=	data[i];												//������ֵ
	}
	OSIntExit();

	/*���ͱ���*/

	m_can.mbox		=	CAN_Transmit(CAN1,(CanTxMsg*)msg);							//CAN�������������䣬���ͺ����᷵��������ţ���mboxֵ
	if(CAN_NO_MB == m_can.mbox)
	{
		return	0;																	//���û�п��õķ������䣬����0
	}
	else
	{
		m_can.CAN_msg_num[m_can.mbox]	=	1;
	}
	
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
	return	1;
}



	/*---------------------------------------------------------------------------
	��������			��BrakeControl(void)
	��������			��mode:		1 ������բ		0 �رձ�բ		���� �޲���
	��������			�������رձ�բʹ�ú���
	----------------------------------------------------------------------------*/
void	BrakeControl(uint8_t	mode)
{
//	uint8_t 	cnt = 0;
//	uint32_t	io_state;
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
	��������			��LED_GREEN_ON(void)
	��������			��
	��������			����ɫLED����
	----------------------------------------------------------------------------*/
void	LED_GREEN_ON(void)
{
	GPIOC->ODR |= 0x0002;
}

	/*---------------------------------------------------------------------------
	��������			��LED_GREEN_OFF(void)
	��������			��
	��������			����ɫLEDϨ��
	----------------------------------------------------------------------------*/
void	LED_GREEN_OFF(void)
{
	GPIOC->ODR &= 0xFFFD;
}


	/*---------------------------------------------------------------------------
	��������			��LED_RED_ON(void)
	��������			��
	��������			����ɫLED����
	----------------------------------------------------------------------------*/
void	LED_RED_ON(void)
{
	GPIOC->ODR |= 0x0004;
}

	/*---------------------------------------------------------------------------
	��������			��LED_RED_OFF(void)
	��������			��
	��������			����ɫLEDϨ��
	----------------------------------------------------------------------------*/
void	LED_RED_OFF(void)
{
	GPIOC->ODR &= 0xFFFB;
}

/**********************end of file************************/
