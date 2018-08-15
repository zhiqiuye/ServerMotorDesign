/**
  ******************************************************************************
  * @file    Project/user/current_filter.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20180808
  * @brief   �����˲�����
  ******************************************************************************
  ******************************************************************************
  */

#include	"current_filter.h"
#include	"global_parameters.h"
#include	"stm32f4xx.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
//�����˲��� ��ֵ
#define		NOISE_PULSATION_THRESHOLD					500



/* Private variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
uint16_t	Single_Current_Average_X4_Filter(uint8_t channel,uint16_t * new_data);
uint16_t	Single_Current_Average_X8_Filter(uint8_t channel,uint16_t * new_data);


/* Private functions ---------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	��������			��Current_Filter_Init(void)
	��������			��
	��������			�����˲����ڲ����ݳ�ʼ��Ϊ0
	----------------------------------------------------------------------------*/
void	Current_Filter_Init(void)
{
	uint8_t	i=3;
	while(i--)
	{
		m_motor_rt_para.u16_uvw_sum[i]		=	0;
		m_motor_rt_para.filter_index_uvw[i]	=	0;
		m_motor_rt_para.uvw_buffer_used[i]	=	0;
	}
	m_motor_rt_para.u16_u_curr_bias			=	0;
	m_motor_rt_para.u16_v_curr_bias			=	0;
	m_motor_rt_para.u16_w_curr_bias			=	0;
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
void	Current_Average_X4_Filter(motor_runtime_para * m_parg)
{
	m_parg->u16_u_current	=	Single_Current_Average_X4_Filter(0,&(m_parg->ADC_DMA_buf[0]));
	m_parg->u16_v_current	=	Single_Current_Average_X4_Filter(1,&(m_parg->ADC_DMA_buf[1]));
	m_parg->u16_w_current	=	Single_Current_Average_X4_Filter(2,&(m_parg->ADC_DMA_buf[2]));
	if(m_sys_state.u8_cur_state == Run_state)
	{
		if(m_parg->u16_u_current < m_parg->u16_u_curr_bias)
			m_parg->u16_u_current = m_parg->u16_u_curr_bias;									//����û�и�ֵ�����Բ�������ֵӦ����ƫ��ֵ
		if(m_parg->u16_v_current < m_parg->u16_v_curr_bias)
			m_parg->u16_v_current = m_parg->u16_v_curr_bias;	
		if(m_parg->u16_w_current < m_parg->u16_w_curr_bias)
			m_parg->u16_w_current = m_parg->u16_w_curr_bias;
	}
		
}

	/*---------------------------------------------------------------------------
	��������			��Single_Current_Average_X4_Filter(uint8_t channel,uint16_t * new_data)
	��������			��	uint8_t channel		ͨ��UVW
							uint16_t * new_data	�»�ȡ����ָ��

	��������			��  ��ͨ������ֵ���д���ƽ���˲����˲����ڿ��Ϊ4��
							����ֵ����ǰ��ֵ��ֵ������
							�����˲�ǰ�����ݴ����history_data��
							channel 0-2 ��Ӧ UVW
							��ֵ�趨	NOISE_PULSATION_THRESHOLD
							�ڳ�ʼbuffer���ʱ����������ֵ���⣬�����һ���������������������˲���ʧ��
	----------------------------------------------------------------------------*/

uint16_t	Single_Current_Average_X4_Filter(uint8_t channel,uint16_t * new_data)
{
	int16_t		delta_value;											//����������һ�β������ݵĲ�ֵ
	int16_t		pre_value;
	uint16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																			//�����ԭʼ����ֵ���ܳ���4095
	
	if(m_motor_rt_para.uvw_buffer_used[channel] < 4)																//bufferδ����
	{
		m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]]	=	*new_data;			//�������ݼ���buffer
		m_motor_rt_para.uvw_buffer_used[channel]++;
		m_motor_rt_para.filter_index_uvw[channel] 							=	m_motor_rt_para.uvw_buffer_used[channel];			
		m_motor_rt_para.u16_uvw_sum[channel]								+=	*new_data;							//���������������
		return		m_motor_rt_para.u16_uvw_sum[channel]>>2;														//����ƽ��ֵ
	}
	else																											//buffer��װ��
	{
		m_motor_rt_para.filter_index_uvw[channel] ++;																//����Ǩ��
		if(m_motor_rt_para.filter_index_uvw[channel] >= 4)
			m_motor_rt_para.filter_index_uvw[channel] =0;
		
		if(m_motor_rt_para.filter_index_uvw[channel] == 3)															//�Ӵ��ڻ�����ֵ
			abandoned_value			=	m_motor_rt_para.history_data[channel][0];
		else
			abandoned_value			=	m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]+1];
		
		if(m_motor_rt_para.filter_index_uvw[channel] == 0)															//��һ������
			pre_value				=	(int16_t)m_motor_rt_para.history_data[channel][3];
		else
			pre_value				=	(int16_t)m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel] - 1];
		
		delta_value					=	(int16_t)*new_data - pre_value;												//���������������ϴ����ݵĲ�ֵ
		
		if((delta_value > NOISE_PULSATION_THRESHOLD) ||(delta_value < -NOISE_PULSATION_THRESHOLD))
			m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]]	=	pre_value;		//��ֵ��ǰ�β���ֵ֮�������ֵ������
		else
			m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]]	=	*new_data;
		
		m_motor_rt_para.u16_uvw_sum[channel]		+=	m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]];
		m_motor_rt_para.u16_uvw_sum[channel]		-=	abandoned_value;
		
		return	m_motor_rt_para.u16_uvw_sum[channel]>>2;
	}
	
}


	/*---------------------------------------------------------------------------
	��������			��Current_Average_X8_Filter(motor_runtime_para * m_parg)
	��������			��
	��������			������ֵ���д���ƽ���˲����˲����ڿ��Ϊ8
	----------------------------------------------------------------------------*/
void	Current_Average_X8_Filter(motor_runtime_para * m_parg)
{
	m_parg->u16_u_current	=	Single_Current_Average_X8_Filter(0,&(m_parg->ADC_DMA_buf[0]));
	m_parg->u16_v_current	=	Single_Current_Average_X8_Filter(1,&(m_parg->ADC_DMA_buf[1]));
	m_parg->u16_w_current	=	Single_Current_Average_X8_Filter(2,&(m_parg->ADC_DMA_buf[2]));
}


uint16_t	Single_Current_Average_X8_Filter(uint8_t channel,uint16_t * new_data)
{
	int16_t		delta_value;											//����������һ�β������ݵĲ�ֵ
	int16_t		pre_value;
	uint16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																			//�����ԭʼ����ֵ���ܳ���4095
	
	if(m_motor_rt_para.uvw_buffer_used[channel] < 8)																//bufferδ����
	{
		m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]]	=	*new_data;							//�������ݼ���buffer
		m_motor_rt_para.uvw_buffer_used[channel]++;
		m_motor_rt_para.filter_index_uvw[channel] 							=	m_motor_rt_para.uvw_buffer_used[channel];			
		m_motor_rt_para.u16_uvw_sum[channel]								+=	*new_data;							//���������������
		return		m_motor_rt_para.u16_uvw_sum[channel]>>3;														//����ƽ��ֵ
	}
	else																							//buffer��װ��
	{
		m_motor_rt_para.filter_index_uvw[channel] ++;																//����Ǩ��
		if(m_motor_rt_para.filter_index_uvw[channel] >= 8)
			m_motor_rt_para.filter_index_uvw[channel] =0;
		
		if(m_motor_rt_para.filter_index_uvw[channel] == 7)															//�Ӵ��ڻ�����ֵ
			abandoned_value			=	m_motor_rt_para.history_data[channel][0];
		else
			abandoned_value			=	m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]+1];
		
		if(m_motor_rt_para.filter_index_uvw[channel] == 0)															//��һ������
			pre_value				=	(int16_t)m_motor_rt_para.history_data[channel][7];
		else
			pre_value				=	(int16_t)m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel] - 1];
		
		delta_value					=	(int16_t)*new_data - pre_value;								//���������������ϴ����ݵĲ�ֵ
		
		if((delta_value > NOISE_PULSATION_THRESHOLD) ||(delta_value < -NOISE_PULSATION_THRESHOLD))
			m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]]	=	pre_value;						//��ֵ��ǰ�β���ֵ֮�������ֵ������
		else
			m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]]	=	*new_data;
		
		m_motor_rt_para.u16_uvw_sum[channel]		+=	m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]];
		m_motor_rt_para.u16_uvw_sum[channel]		-=	abandoned_value;
		
		return	m_motor_rt_para.u16_uvw_sum[channel]>>3;
	}
}







/*********************************** END OF FILE*****************************************/

