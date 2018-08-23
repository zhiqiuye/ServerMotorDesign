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
#include	"stm32f4xx_dma.h"
#include	"global_parameters.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

float	Speed_Average_X4_Filter(int32_t * new_data, int32_t * buffer, uint8_t * buf_used, int32_t * sum)
{
	float result = 0;
	
	if(*buf_used < 4)										//��ֵ�˲���δ����
	{
		(*buf_used) ++;
		*sum 			+= *new_data;
		result			=	(float)(*sum) / (float)(*buf_used);
	}
	else
	{
		*sum			-=	*buffer;
		*(buffer)		=	*(buffer+1);
		*(buffer+1)		=	*(buffer+2);
		*(buffer+2)		=	*(buffer+3);
		*(buffer+3)		=	*new_data;
		*sum			+=	*new_data;
		result			=	(float)(*sum)/4.0f;
	}
	return	result;
}


	/*---------------------------------------------------------------------------
	��������			��Read_IncEncoder(int32_t * encoder_num)
	��������			��
	��������			��	��ȡ�������������ݣ����µ�ȫ�ֱ�����
	----------------------------------------------------------------------------*/
void	Read_IncEncoder(void)
{
	int32_t		temp_delta;
	float		f_temp	=	0.0f;
	uint32_t	f_width	=	0;
	
	m_motor_rt_para.m_encoder.u16_encoder_last_read		=	m_motor_rt_para.m_encoder.u16_encoder_curr_read;						//��ȡ���±���������
	m_motor_rt_para.m_encoder.u16_encoder_curr_read		=	TIM3->CNT;
	
	temp_delta	=	(int32_t)(m_motor_rt_para.m_encoder.u16_encoder_curr_read - m_motor_rt_para.m_encoder.u16_encoder_last_read);
	
	m_motor_rt_para.m_encoder.i32_pulse_cnt				+=	temp_delta;																//��ȡ��ǰλ����Ϣ
	
	/*����temp_dalta�ж�ת�ٷ���*/
	if(temp_delta < 0)
		m_motor_rt_para.m_encoder.u8_velocity_sign		=	0;
	else
		m_motor_rt_para.m_encoder.u8_velocity_sign		=	1;
	
	/*��ʱ�ر�dma*/
	TIM_DMACmd(TIM5,TIM_DMA_CC1,DISABLE);
	
	/*�������ֵ*/
	
	
	/*M/T���Զ�ת�� f_shaft_cal_speed*/
	if(m_motor_rt_para.m_encoder.u8_M_or_T == M_METHORD)
	{
		/*ʹ��M��ʱ���������ٶȲ�������12.5rpsʱ������T��*/
		if(((m_motor_rt_para.m_encoder.f_motor_cal_speed < 12.5f) && (m_motor_rt_para.m_encoder.f_motor_cal_speed > -12.5f)) && (m_motor_rt_para.m_encoder.u32_pulse_width_buf[1] != 0))
		{
			/*T�����٣����Ǽ�ʱ*/
			// ������4096�ߣ��ı�Ƶ��Ϊ16384 Hz��T������ʱ������������أ�
			// ��ȡ�������� N����һ����������ʱ��	t = N / 42000000 ��
			// ����תһȦ��ʱ��Ϊ�� t2 = t*16384 / 2 
			// ����ת��Ϊ��1/t2 =	5126.953125 / N
			if(m_motor_rt_para.m_encoder.u8_velocity_sign == 1)
				m_motor_rt_para.m_encoder.f_motor_cal_speed		=	5126.953125f/((float)m_motor_rt_para.m_encoder.u32_pulse_width_buf[1]);
			else
				m_motor_rt_para.m_encoder.f_motor_cal_speed		=	-5126.953125f/((float)m_motor_rt_para.m_encoder.u32_pulse_width_buf[1]);
			m_motor_rt_para.m_encoder.u8_M_or_T					=	T_METHORD;
		}
		else
		{
//			f_temp		=	Speed_Average_X4_Filter(&temp_delta, 
//													&m_motor_rt_para.m_encoder.i32_spd_hisdata[0],
//													&m_motor_rt_para.m_encoder.u8_speed_filter_used,
//													&m_motor_rt_para.m_encoder.i32_spd_his_sum);														//�ٶȾ�ֵ�˲�	
			f_temp		=	(float)temp_delta;		
			/*M�����٣���ʱ���*/
			m_motor_rt_para.m_encoder.f_motor_cal_speed			=	f_temp / 16.384f;				
		}
	}
	else
	{
		/*ʹ��T��ʱ���������ٶȲ�������12.5rpsʱ������M��*/
		if(((m_motor_rt_para.m_encoder.f_motor_cal_speed > 12.5f) && (m_motor_rt_para.m_encoder.f_motor_cal_speed < -12.5f)) || (m_motor_rt_para.m_encoder.u32_pulse_width_buf[1] == 0) )
		{
			f_temp		=	(float)temp_delta;		
			/*M�����٣���ʱ���*/
			m_motor_rt_para.m_encoder.f_motor_cal_speed			=	f_temp / 16.384f;
			m_motor_rt_para.m_encoder.u8_M_or_T					=	M_METHORD;
		}
		/*T�����٣����Ǽ�ʱ*/
		else
		{			
			if(m_motor_rt_para.m_encoder.u8_velocity_sign == 1)
				m_motor_rt_para.m_encoder.f_motor_cal_speed		=	5126.953125f/((float)m_motor_rt_para.m_encoder.u32_pulse_width_buf[1]);
			else
				m_motor_rt_para.m_encoder.f_motor_cal_speed		=	-5126.953125f/((float)m_motor_rt_para.m_encoder.u32_pulse_width_buf[1]);
		}
	}
	/*����DMAͨ��*/
	TIM_DMACmd(TIM5,TIM_DMA_CC1,ENABLE);
	
	m_motor_ctrl.u8_speed_read_data_refreshed					=	1;
}








/*****************************END OF FILE************************************/

