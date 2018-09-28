/**
  ******************************************************************************
  * @file    Project/ucosii stm32f4 mdk475/main.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20161010
  * @brief   Main program body
  ******************************************************************************

  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include	"stm32f4xx.h"
#include	"stm32f4xx_gpio.h"
#include	"stm32f4xx_tim.h"
#include	"stm32f4xx_rcc.h"
#include	"math.h"
#include	"arm_math.h"
#include	"includes.h"
#include	"peripherial_init.h"
#include	"app.h"
#include	"global_parameters.h"
#include	"stm32f4xx_it.h"
#include	"motor_control.h"
#include	"delay.h"
#include	"hall_reversal_6steps.h"



/* Private function prototypes -----------------------------------------------*/
static void	Board_Init(void);
static void APP_TaskStart(void *p_arg);
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
OS_EVENT	*sem_USART2Rev,
			*sem_USART3Rev;


/* Private macro -------------------------------------------------------------*/
#define	StateMachine_TASK_PRIO										6
#define StateMachine_TASK_STK_SIZE									256
OS_STK	StateMachine_TASK_STK[StateMachine_TASK_STK_SIZE];

#define	RS485_TASK_PRIO												7
#define	RS485_TASK_STK_SIZE											128
OS_STK	RS485_TASK_STK[RS485_TASK_STK_SIZE];

#define	LED_TASK_PRIO												49
#define	LED_TASK_STK_SIZE											64
OS_STK	LED_TASK_STK[LED_TASK_STK_SIZE];

#define	APP_TASK_START_PRIO											50
#define	APP_TASK_START_SIZE											256
OS_STK	APP_TASK_START_STK[APP_TASK_START_SIZE];
/* Private variables ---------------------------------------------------------*/
int main()
{
/*使用中断向量组2，抢占优先级为4，相应优先级为4*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
/*电机参数初始化*/
	ParametersInit();

/*芯片外设初始化*/
	Board_Init();	
	
/*进入操作系统*/
	OSInit();
	
/*创建信号量*/
	sem_USART2Rev	=	OSSemCreate(0);
	sem_USART3Rev	=	OSSemCreate(0);
	
/*创建起始任务*/
	OSTaskCreateExt(APP_TaskStart,
					(void*)0,
					(OS_STK *)&APP_TASK_START_STK[APP_TASK_START_SIZE-1],
					APP_TASK_START_PRIO,
					APP_TASK_START_PRIO,
					(OS_STK *)&APP_TASK_START_STK[0],
					APP_TASK_START_SIZE,
					0,
					OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
					
/*启动uCOSII*/
	OSStart();
}



/* Private functions ---------------------------------------------------------*/
/******************************************************************************
*			函数说明：	起始任务，创建其他任务后挂起
*			参数说明：	无
*			使用范围：	本文件内
*******************************************************************************/
static void APP_TaskStart(void *p_arg)
{
/*设置系统时基*/
	OS_CPU_SysTickInit(168000000/OS_TICKS_PER_SEC);								//时基10ms,在启动多任务后开启系统时钟,OS_TICKS_PER_SEC
	delay_us(500);
	
/*系统状态监测初始化*/
	OSStatInit();
	
/*建立各任务*/
	
	//状态机管理
	OSTaskCreateExt(StateMachine_Task,
				(void *)0,
				(OS_STK *)&StateMachine_TASK_STK[StateMachine_TASK_STK_SIZE-1],
				StateMachine_TASK_PRIO,
				StateMachine_TASK_PRIO,
				(OS_STK *)&StateMachine_TASK_STK[0],
				StateMachine_TASK_STK_SIZE,
				0,
				OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);	
				
	//LED闪烁
	OSTaskCreateExt(LED_Task,
				(void *)0,
				(OS_STK *)&LED_TASK_STK[LED_TASK_STK_SIZE-1],
				LED_TASK_PRIO,
				LED_TASK_PRIO,
				(OS_STK *)&LED_TASK_STK[0],
				LED_TASK_STK_SIZE,
				0,
				OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);
				
	//485通讯
	OSTaskCreateExt(RS485_Task,
				(void *)0,
				(OS_STK *)&RS485_TASK_STK[RS485_TASK_STK_SIZE-1],
				RS485_TASK_PRIO,
				RS485_TASK_PRIO,
				(OS_STK *)&RS485_TASK_STK[0],
				RS485_TASK_STK_SIZE,
				0,
				OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);		
	

/*删除自身*/
	OSTaskDel(OS_PRIO_SELF);
}






/******************************************************************************
*			函数说明：	STM32外设初始化
*			参数说明：	无
*			使用范围：	本文件内
*******************************************************************************/
static void	Board_Init(void)
{
	NVIC_Config();									//中断优先级配置
	GPIO_Config();									//端口配置
	
	SPI1_DMA_Config();								//绝对值编码器接口
	Timer1_Config();								//PWM生成/电流环定时器配置				配置时已打开
	Timer2_Config();								//产生1KHz的中断，进行速度位置环调节	配置时关闭
	Timer3_Config();								//正交编码器定时器配置，M法测量			配置时已打开
	Timer4_Config();								//Hall传感器输入捕获中断				配置时已打开
	Timer5_Config();								//高速端光电编码器低速测量 T法测量		配置时开启
	Timer8_Master_Config();							//定时器9作为TIM2的输入，				配置时关闭
	ADC1_DMA_Config();								//ADC2采集DMA使能，采集相电流
	DAC_Config();
	CAN_Config();
	
	//部分参数的获取与设置
	m_motor_rt_para.m_reverse.u8_hall_state		=	Hall_State_Read();								//立刻知道电机相位，后面滤波器初始化需要用到
	BrakeControl(1);
	m_motor_ctrl.m_sys_state.u8_cur_state		=	Prepare_state;
	m_motor_ctrl.m_sys_state.u8_pre_state		=	Idle_state;
}






/************************ Jingle *******************************END OF FILE****/


