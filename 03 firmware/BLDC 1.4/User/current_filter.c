/**
  ******************************************************************************
  * @file    Project/user/current_filter.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20180808
  * @brief   �����˲�����
  *
  *				��bldc����ֵ���������У�ʹ���������ʵ��������ű۲�����ÿ������⵽
  *			��ͨ·���ű۵�ͨʱ�ĵ���ֵ����ˣ���Ҫ֪����ǰ��ͨ��λ��Ȼ��ȡ����Ӧ��
  *			��ͨ������Ϊ�������
  *				��ʼ�������˲���ʱ������·�����ֱ���д���ƽ���˲���������ζ�Ž���
  *			·����ʱ�ĵ����仯���뵽���ջ�õ�������С�
  *				��ȷ����Ӧ����ÿ��ѡ����ȷ��һ·�����ԭʼֵ�����˲���
  ******************************************************************************
  ******************************************************************************
  */

#include	"current_filter.h"
#include	"global_parameters.h"
#include	"stm32f4xx.h"
#include	"math.h"
#include	"hall_reversal_6steps.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
//�����˲��� ��ֵ
#define		NOISE_PULSATION_THRESHOLD					50



/* Private variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
uint16_t	Single_Current_Average_X4_Filter(uint16_t * new_data);
int16_t		Single_Current_Average_X8_Filter(int16_t * new_data);


/* Private functions ---------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	��������			��Current_Filter_Init(void)
	��������			��
	��������			�����˲����ڲ����ݳ�ʼ��Ϊ0
	----------------------------------------------------------------------------*/
void	Current_Filter_Init(void)
{
	m_motor_rt_para.m_current_filter.i16_uvw_sum				=	0;
	m_motor_rt_para.m_current_filter.filter_index_uvw			=	0;
	m_motor_rt_para.m_current_filter.uvw_buffer_used			=	0;
	m_motor_rt_para.m_current_sensor.u8_curr_bias_ready			=	0;
	m_motor_rt_para.m_current_sensor.i16_uvw_curr_bias			=	0;
}


	/*---------------------------------------------------------------------------
	��������			��Current_Average_X4_Filter(motor_runtime_para * m_parg)
	��������			��
	��������			������ֵ���д���ƽ���˲����˲����ڿ��Ϊ4��
							����ֵ����ƽ��ֵ��ֵ������
							�����˲�ǰ�����ݴ����m_motor_rt_para.ADC_DMA_buf��
							ADC_DMA_buf 0-2 ��Ӧ UVW
							��·���ݼ����ʱ0.8us��pwm����50us
	----------------------------------------------------------------------------*/
void	Current_Average_X4_Filter(current_sensor_state * m_parg)
{
	int16_t		u_i,v_i,w_i;
	uint16_t	real_i;
	/*����u���������ֵ�����ͱ���*/
	u_i							=	m_parg->ADC_DMA_buf[0] - m_parg->i16_uvw_curr_bias;
	u_i							=	((u_i) > 0 ? (u_i) : -(u_i) );
	/*����u���������ֵ�����ͱ���*/
	v_i							=	m_parg->ADC_DMA_buf[1] - m_parg->i16_uvw_curr_bias;
	v_i							=	((v_i) > 0 ? (v_i) : -(v_i) );
	/*����u���������ֵ�����ͱ���*/
	w_i							=	m_parg->ADC_DMA_buf[2] - m_parg->i16_uvw_curr_bias;
	w_i							=	((w_i) > 0 ? (w_i) : -(w_i) );	
	/*��������ƽ��*/
	real_i						=	(u_i + v_i + w_i)>>1;
	/*ѹ���˲���*/
	m_parg->i16_uvw_current		=	Single_Current_Average_X4_Filter(&real_i);
	
	if(m_motor_ctrl.m_sys_state.u8_cur_state == Run_state)	
	{
		if(m_parg->i16_uvw_current > 2000)
			m_parg->i16_uvw_current = 2000;												//���õ���ֵ����
	}
}

	/*---------------------------------------------------------------------------
	��������			��Single_Current_Average_X4_Filter(uint16_t * new_data)
	��������			��	
							uint16_t * new_data	�»�ȡ����ָ��

	��������			��  ѡ����Ӧ��ͨ�����ݣ������databuf�н���ƽ��
	----------------------------------------------------------------------------*/

uint16_t	Single_Current_Average_X4_Filter(uint16_t * new_data)
{
//	int16_t		delta_value;																											//����������һ�β������ݵĲ�ֵ
//	int16_t		pre_value;
	uint16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																												//�����ԭʼ����ֵ���ܳ���4095
	
	if(m_motor_rt_para.m_current_filter.uvw_buffer_used < 4)																			//bufferδ����
	{
		m_motor_rt_para.m_current_filter.history_data[m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*new_data;				//�������ݼ���buffer
		m_motor_rt_para.m_current_filter.uvw_buffer_used++;
		m_motor_rt_para.m_current_filter.filter_index_uvw 								=	m_motor_rt_para.m_current_filter.uvw_buffer_used;			
		m_motor_rt_para.m_current_filter.i16_uvw_sum									+=	*new_data;									//���������������
		return		m_motor_rt_para.m_current_filter.i16_uvw_sum>>2;																	//����ƽ��ֵ
	}
	else																																//buffer��װ��
	{
		m_motor_rt_para.m_current_filter.filter_index_uvw++;																			//����Ǩ��
		if(m_motor_rt_para.m_current_filter.filter_index_uvw >= 4)
			m_motor_rt_para.m_current_filter.filter_index_uvw =0;
		
		if(m_motor_rt_para.m_current_filter.filter_index_uvw == 3)																		//�Ӵ��ڻ�����ֵ
			abandoned_value			=	m_motor_rt_para.m_current_filter.history_data[0];
		else
			abandoned_value			=	m_motor_rt_para.m_current_filter.history_data[m_motor_rt_para.m_current_filter.filter_index_uvw + 1];
		
		m_motor_rt_para.m_current_filter.history_data[m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*new_data;
		
		m_motor_rt_para.m_current_filter.i16_uvw_sum		+=	m_motor_rt_para.m_current_filter.history_data[m_motor_rt_para.m_current_filter.filter_index_uvw];
		m_motor_rt_para.m_current_filter.i16_uvw_sum		-=	abandoned_value;
		
		return	m_motor_rt_para.m_current_filter.i16_uvw_sum>>2;
	}
	
}


	/*---------------------------------------------------------------------------
	��������			��Current_Average_X8_Filter(motor_runtime_para * m_parg)
	��������			��
	��������			������ֵ���д���ƽ���˲����˲����ڿ��Ϊ8
	----------------------------------------------------------------------------*/
void	Current_Average_X8_Filter(current_sensor_state * m_parg)
{
//	int16_t		u_i,v_i,w_i;
	int16_t		real_i;
//	/*����u���������ֵ�����ͱ���*/
//	u_i							=	m_parg->ADC_DMA_buf[0] - m_parg->i16_uvw_curr_bias;	
//	u_i							=	((u_i) > 0 ? (u_i) : -(u_i) );//((m_parg->ADC_DMA_buf[0]>m_parg->i16_uvw_curr_bias)?(u_i) : -(u_i));//
//	/*����v���������ֵ�����ͱ���*/
//	v_i							=	m_parg->ADC_DMA_buf[1] - m_parg->i16_uvw_curr_bias;
//	v_i							=	((v_i) > 0 ? (v_i) : -(v_i) );
//	/*����w���������ֵ�����ͱ���*/
//	w_i							=	m_parg->ADC_DMA_buf[2] - m_parg->i16_uvw_curr_bias;
//	w_i							=	((w_i) > 0 ? (w_i) : -(w_i) );	
//	/*��������ƽ��*/
//	real_i						=	(u_i + v_i + w_i)>>1;
//	/*ѹ���˲���*/
//	m_parg->i16_uvw_current		=	Single_Current_Average_X8_Filter(&real_i);
	//------------------------------------------------------------------------
	if(m_motor_ctrl.m_motion_ctrl.u8_dir	== 	0)
	{
		switch (m_motor_rt_para.m_reverse.u8_hall_state)
		{
			case 1: real_i = m_parg->adc_dma_shadow[2] - (int16_t)m_parg->i16_uvw_curr_bias; break;
			case 2:	real_i = m_parg->adc_dma_shadow[1] - (int16_t)m_parg->i16_uvw_curr_bias; break;
			case 3: real_i = m_parg->adc_dma_shadow[2] - (int16_t)m_parg->i16_uvw_curr_bias; break;
			case 4:	real_i = m_parg->adc_dma_shadow[0] - (int16_t)m_parg->i16_uvw_curr_bias; break;
			case 5: real_i = m_parg->adc_dma_shadow[0] - (int16_t)m_parg->i16_uvw_curr_bias; break;
			case 6:	real_i = m_parg->adc_dma_shadow[1] - (int16_t)m_parg->i16_uvw_curr_bias; break;
		}
	}
	else
	{
		switch (m_motor_rt_para.m_reverse.u8_hall_state)
		{
			case 1: real_i = m_parg->adc_dma_shadow[1] - (int16_t)m_parg->i16_uvw_curr_bias; break;
			case 2:	real_i = m_parg->adc_dma_shadow[0] - (int16_t)m_parg->i16_uvw_curr_bias; break;
			case 3: real_i = m_parg->adc_dma_shadow[0] - (int16_t)m_parg->i16_uvw_curr_bias; break;
			case 4:	real_i = m_parg->adc_dma_shadow[2] - (int16_t)m_parg->i16_uvw_curr_bias; break;
			case 5: real_i = m_parg->adc_dma_shadow[1] - (int16_t)m_parg->i16_uvw_curr_bias; break;
			case 6:	real_i = m_parg->adc_dma_shadow[2] - (int16_t)m_parg->i16_uvw_curr_bias; break;
		}
	}
	m_parg->i16_uvw_current		=	Single_Current_Average_X8_Filter(&real_i);//real_i - (int16_t)m_parg->i16_uvw_curr_bias;
	
	if(m_motor_ctrl.m_sys_state.u8_cur_state == Run_state)	
	{
		if(m_parg->i16_uvw_current > 2000)
			m_parg->i16_uvw_current = 2000;																		//���õ���ֵ����
	}
}


int16_t	Single_Current_Average_X8_Filter(int16_t * new_data)
{
	int16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																						//�����ԭʼ����ֵ���ܳ���4095
	
	if(m_motor_rt_para.m_current_filter.uvw_buffer_used < 8)													//bufferδ����
	{
		m_motor_rt_para.m_current_filter.history_data[m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*new_data;								//�������ݼ���buffer
		m_motor_rt_para.m_current_filter.uvw_buffer_used++;
		m_motor_rt_para.m_current_filter.filter_index_uvw 								=	m_motor_rt_para.m_current_filter.uvw_buffer_used;			
		m_motor_rt_para.m_current_filter.i16_uvw_sum									+=	*new_data;			//���������������
		return		m_motor_rt_para.m_current_filter.i16_uvw_sum>>3;											//����ƽ��ֵ
	}
	else																										//buffer��װ��
	{
		m_motor_rt_para.m_current_filter.filter_index_uvw++;													//����Ǩ��
		if(m_motor_rt_para.m_current_filter.filter_index_uvw >= 8)
			m_motor_rt_para.m_current_filter.filter_index_uvw =0;
		
		if(m_motor_rt_para.m_current_filter.filter_index_uvw == 7)												//�Ӵ��ڻ�����ֵ
			abandoned_value			=	m_motor_rt_para.m_current_filter.history_data[0];
		else
			abandoned_value			=	m_motor_rt_para.m_current_filter.history_data[m_motor_rt_para.m_current_filter.filter_index_uvw + 1];
		
		/*��Ұֵ*/
		if((*new_data - m_motor_rt_para.m_current_sensor.i16_uvw_current) > NOISE_PULSATION_THRESHOLD)
			m_motor_rt_para.m_current_filter.history_data[m_motor_rt_para.m_current_filter.filter_index_uvw]	=	m_motor_rt_para.m_current_sensor.i16_uvw_current;
		else		
			m_motor_rt_para.m_current_filter.history_data[m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*new_data;
		
		m_motor_rt_para.m_current_filter.i16_uvw_sum		+=	m_motor_rt_para.m_current_filter.history_data[m_motor_rt_para.m_current_filter.filter_index_uvw];
		m_motor_rt_para.m_current_filter.i16_uvw_sum		-=	abandoned_value;
		
		return	m_motor_rt_para.m_current_filter.i16_uvw_sum>>3;
	}
}







/*********************************** END OF FILE*****************************************/

