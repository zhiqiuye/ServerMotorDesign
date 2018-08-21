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

float	Speed_Average_X4_Filter(int32_t  new_data)
{
	int32_t		abandoned_data;
	if(m_motor_rt_para.u8_speed_filter_used < 4)					//������ֵ�˲���δ����
	{
		m_motor_rt_para.i32_spd_hisdata[m_motor_rt_para.u8_speed_filter_used]	=	new_data;
		m_motor_rt_para.u8_speed_filter_used++;
		m_motor_rt_para.u8_speed_filter_index									=	m_motor_rt_para.u8_speed_filter_used;
		m_motor_rt_para.i32_spd_his_sum											+=	new_data;
		return	new_data;
	}
	else
	{
		m_motor_rt_para.u8_speed_filter_index++;
		if(m_motor_rt_para.u8_speed_filter_index >= 4)
			m_motor_rt_para.u8_speed_filter_index	=	0;
		
		if(m_motor_rt_para.u8_speed_filter_index ==	3)
			abandoned_data	=	m_motor_rt_para.i32_spd_hisdata[0];
		else
			abandoned_data	=	m_motor_rt_para.i32_spd_hisdata[m_motor_rt_para.u8_speed_filter_index + 1];
		m_motor_rt_para.i32_spd_hisdata[m_motor_rt_para.u8_speed_filter_index]	=	new_data;
		m_motor_rt_para.i32_spd_his_sum	+=	m_motor_rt_para.i32_spd_hisdata[m_motor_rt_para.u8_speed_filter_index];
		m_motor_rt_para.i32_spd_his_sum	-=	abandoned_data;
		return	(float)m_motor_rt_para.i32_spd_his_sum / 4.0f;
	}
}


	/*---------------------------------------------------------------------------
	��������			��Read_IncEncoder(int32_t * encoder_num)
	��������			��
	��������			��	��ȡ�������������ݣ����µ�ȫ�ֱ�����
	----------------------------------------------------------------------------*/
void	Read_IncEncoder(void)
{
	int16_t		temp_delta;
	float		f_temp	=	0.0f;
	
	m_motor_rt_para.u16_encoder_last_read		=	m_motor_rt_para.u16_encoder_curr_read;						//��ȡ���±���������
	m_motor_rt_para.u16_encoder_curr_read		=	TIM3->CNT;
	
	temp_delta	=	(int16_t)(m_motor_rt_para.u16_encoder_curr_read - m_motor_rt_para.u16_encoder_last_read);
	
	m_motor_rt_para.i32_pulse_cnt				+=	(int32_t)temp_delta;										//��ȡ��ǰλ����Ϣ
	
	f_temp		=	(float)temp_delta;//Speed_Average_X4_Filter((int32_t)temp_delta);//														//�ٶȾ�ֵ�˲�	
	
	m_motor_rt_para.f_motor_cal_speed			=	f_temp / 40.0f;
	
	m_motor_ctrl.u8_speed_read_data_refreshed	=	1;
}








/*****************************END OF FILE************************************/

