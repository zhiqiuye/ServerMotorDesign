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



/* Private function prototypes -----------------------------------------------*/
static void	Board_Init(void);
static void APP_TaskStart(void *p_arg);
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
OS_EVENT	*sem_USART2Rev,
			*sem_USART3Rev;


/* Private macro -------------------------------------------------------------*/
#define	RS485_TASK_PRIO												6
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
	
/*芯片外设初始化*/
	Board_Init();
	
/*电机参数初始化*/
	ParametersInit();	
	
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
	OSTimeDlyHMSM(0,0,0,500);
	
/*建立各任务*/
	
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
				
	//LED闪烁
	OSTaskCreateExt(RS485_Task,
				(void *)0,
				(OS_STK *)&RS485_TASK_STK[RS485_TASK_STK_SIZE-1],
				RS485_TASK_PRIO,
				RS485_TASK_PRIO,
				(OS_STK *)&RS485_TASK_STK[0],
				RS485_TASK_STK_SIZE,
				0,
				OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);		

	BrakeControl(1);																//开启抱闸
			
	//任务开始前只进行了板级外设初始化，并未开启各种功能
	
	Hall_State_Read();											//检测初始传感器相位
	Hall_Convert();												//开始初次相位识别					

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
	
	Timer1_Config();								//PWM生成定时器配置
//	Timer2_Config();								//产生20KHz的中断，进行电流环调节
	Timer3_Config();								//正交编码器定时器配置
	Timer4_Config();								//Hall传感器输入捕获中断
	ADC_DMA_Config();								//ADC采集DMA使能
	
	
}






/************************ Jingle *******************************END OF FILE****/


