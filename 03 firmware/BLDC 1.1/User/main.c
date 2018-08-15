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
/*ʹ���ж�������2����ռ���ȼ�Ϊ4����Ӧ���ȼ�Ϊ4*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
/*оƬ�����ʼ��*/
	Board_Init();
	
/*���������ʼ��*/
	ParametersInit();	
	
/*�������ϵͳ*/
	OSInit();
	
/*�����ź���*/
	sem_USART2Rev	=	OSSemCreate(0);
	sem_USART3Rev	=	OSSemCreate(0);
	
/*������ʼ����*/
	OSTaskCreateExt(APP_TaskStart,
					(void*)0,
					(OS_STK *)&APP_TASK_START_STK[APP_TASK_START_SIZE-1],
					APP_TASK_START_PRIO,
					APP_TASK_START_PRIO,
					(OS_STK *)&APP_TASK_START_STK[0],
					APP_TASK_START_SIZE,
					0,
					OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
					
/*����uCOSII*/
	OSStart();
}



/* Private functions ---------------------------------------------------------*/
/******************************************************************************
*			����˵����	��ʼ���񣬴���������������
*			����˵����	��
*			ʹ�÷�Χ��	���ļ���
*******************************************************************************/
static void APP_TaskStart(void *p_arg)
{
/*����ϵͳʱ��*/
	OS_CPU_SysTickInit(168000000/OS_TICKS_PER_SEC);								//ʱ��10ms,���������������ϵͳʱ��,OS_TICKS_PER_SEC
	OSTimeDlyHMSM(0,0,0,500);
	
/*����������*/
	
	//LED��˸
	OSTaskCreateExt(LED_Task,
				(void *)0,
				(OS_STK *)&LED_TASK_STK[LED_TASK_STK_SIZE-1],
				LED_TASK_PRIO,
				LED_TASK_PRIO,
				(OS_STK *)&LED_TASK_STK[0],
				LED_TASK_STK_SIZE,
				0,
				OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);
				
	//LED��˸
	OSTaskCreateExt(RS485_Task,
				(void *)0,
				(OS_STK *)&RS485_TASK_STK[RS485_TASK_STK_SIZE-1],
				RS485_TASK_PRIO,
				RS485_TASK_PRIO,
				(OS_STK *)&RS485_TASK_STK[0],
				RS485_TASK_STK_SIZE,
				0,
				OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);		

	BrakeControl(1);																//������բ
			
	//����ʼǰֻ�����˰弶�����ʼ������δ�������ֹ���
	
	Hall_State_Read();											//����ʼ��������λ
	Hall_Convert();												//��ʼ������λʶ��					

/*ɾ������*/
	OSTaskDel(OS_PRIO_SELF);
}






/******************************************************************************
*			����˵����	STM32�����ʼ��
*			����˵����	��
*			ʹ�÷�Χ��	���ļ���
*******************************************************************************/
static void	Board_Init(void)
{
	NVIC_Config();									//�ж����ȼ�����
	GPIO_Config();									//�˿�����
	
	Timer1_Config();								//PWM���ɶ�ʱ������
//	Timer2_Config();								//����20KHz���жϣ����е���������
	Timer3_Config();								//������������ʱ������
	Timer4_Config();								//Hall���������벶���ж�
	ADC_DMA_Config();								//ADC�ɼ�DMAʹ��
	
	
}






/************************ Jingle *******************************END OF FILE****/


