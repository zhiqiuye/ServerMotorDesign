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
/*ʹ���ж�������2����ռ���ȼ�Ϊ4����Ӧ���ȼ�Ϊ4*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
/*���������ʼ��*/
	ParametersInit();

/*оƬ�����ʼ��*/
	Board_Init();	
	
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
	delay_us(500);
	
/*ϵͳ״̬����ʼ��*/
	OSStatInit();
	
/*����������*/
	
	//״̬������
	OSTaskCreateExt(StateMachine_Task,
				(void *)0,
				(OS_STK *)&StateMachine_TASK_STK[StateMachine_TASK_STK_SIZE-1],
				StateMachine_TASK_PRIO,
				StateMachine_TASK_PRIO,
				(OS_STK *)&StateMachine_TASK_STK[0],
				StateMachine_TASK_STK_SIZE,
				0,
				OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);	
				
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
				
	//485ͨѶ
	OSTaskCreateExt(RS485_Task,
				(void *)0,
				(OS_STK *)&RS485_TASK_STK[RS485_TASK_STK_SIZE-1],
				RS485_TASK_PRIO,
				RS485_TASK_PRIO,
				(OS_STK *)&RS485_TASK_STK[0],
				RS485_TASK_STK_SIZE,
				0,
				OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);		
	

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
	
	SPI1_DMA_Config();								//����ֵ�������ӿ�
	Timer1_Config();								//PWM����/��������ʱ������				����ʱ�Ѵ�
	Timer2_Config();								//����1KHz���жϣ������ٶ�λ�û�����	����ʱ�ر�
	Timer3_Config();								//������������ʱ�����ã�M������			����ʱ�Ѵ�
	Timer4_Config();								//Hall���������벶���ж�				����ʱ�Ѵ�
	Timer5_Config();								//���ٶ˹����������ٲ��� T������		����ʱ����
	Timer8_Master_Config();							//��ʱ��9��ΪTIM2�����룬				����ʱ�ر�
	ADC1_DMA_Config();								//ADC2�ɼ�DMAʹ�ܣ��ɼ������
	DAC_Config();
	CAN_Config();
	
	//���ֲ����Ļ�ȡ������
	m_motor_rt_para.m_reverse.u8_hall_state		=	Hall_State_Read();								//����֪�������λ�������˲�����ʼ����Ҫ�õ�
	BrakeControl(1);
	m_motor_ctrl.m_sys_state.u8_cur_state		=	Prepare_state;
	m_motor_ctrl.m_sys_state.u8_pre_state		=	Idle_state;
}






/************************ Jingle *******************************END OF FILE****/


