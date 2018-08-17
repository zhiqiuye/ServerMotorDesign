/**
  ******************************************************************************
  * @file    Project/user/spd_pos_filter.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20180814
  * @brief   ���������ݶ�ȡ���ٶ�λ�ý���
  ******************************************************************************
  ******************************************************************************
  */

#include	"stm32f4xx.h"
#include	"stm32f4xx_tim.h"
#include	"global_parameters.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

	/*---------------------------------------------------------------------------
	��������			��Read_IncEncoder(int32_t * encoder_num)
	��������			��
	��������			��	��ȡ�������������ݣ����µ�ȫ�ֱ�����
	----------------------------------------------------------------------------*/
void	Read_IncEncoder(void)
{
	int32_t		temp_delta;
	
	m_motor_rt_para.u16_encoder_last_read	=	m_motor_rt_para.u16_encoder_curr_read;		//��ȡ���±���������
	m_motor_rt_para.u16_encoder_curr_read	=	TIM3->CNT;
	
	temp_delta	=	(int16_t)(m_motor_rt_para.u16_encoder_curr_read - m_motor_rt_para.u16_encoder_last_read);
	
	m_motor_rt_para.f_motor_cal_speed		=	((float)temp_delta)/40.0f;
	
	
//	if(temp_delta == 0)
//		temp_delta	=	1;
//	if(temp_delta >=100)
//		;
}








/*****************************END OF FILE************************************/

