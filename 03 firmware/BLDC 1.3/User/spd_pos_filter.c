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
	
	m_motor_rt_para.i32_encoder_last_read	=	m_motor_rt_para.i32_encoder_curr_read;		//��ȡ���±���������
	m_motor_rt_para.i32_encoder_curr_read	=	TIM3->CNT;
	
	temp_delta	=	m_motor_rt_para.i32_encoder_curr_read - m_motor_rt_para.i32_encoder_last_read;
	
	if(temp_delta < -40000)						//������������������¼���
	{
		m_motor_rt_para.f_motor_cal_speed	=	((float)(temp_delta + 65535))/40.0f;		///40000*1000 ���rps��λ���ٶ�
	}
	else if(temp_delta > 40000)					//�������������������0
	{
		m_motor_rt_para.f_motor_cal_speed	=	((float)(65535 - temp_delta))/40.0f;		///40000*1000 ���rps��λ���ٶ�
	}
	else									//��������0-65535֮����������
	{
		m_motor_rt_para.f_motor_cal_speed	=	((float)temp_delta)/40.0f;					///40000*1000 ���rps��λ���ٶ�
	}
}








/*****************************END OF FILE************************************/

