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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
//�����˲��� ��ֵ
#define		NOISE_PULSATION_THRESHOLD					300



/* Private variables ---------------------------------------------------------*/
//���������
extern const	uint8_t		current_senser_table[2][7] ;										



/* Private function prototypes -----------------------------------------------*/
uint16_t	Single_Current_Average_X4_Filter(uint16_t * new_data);
uint16_t	Single_Current_Average_X8_Filter(uint16_t * new_data);


/* Private functions ---------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	��������			��Current_Filter_Init(void)
	��������			��
	��������			�����˲����ڲ����ݳ�ʼ��Ϊ0
	----------------------------------------------------------------------------*/
void	Current_Filter_Init(void)
{
	m_motor_rt_para.u16_uvw_sum				=	0;
	m_motor_rt_para.filter_index_uvw		=	0;
	m_motor_rt_para.uvw_buffer_used			=	0;
	m_motor_rt_para.u16_uvw_curr_bias		=	0;
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
	int16_t		u_i,v_i,w_i;
	uint16_t	real_i;
	/*����u���������ֵ�����ͱ���*/
	u_i							=	m_parg->ADC_DMA_buf[0] - m_parg->u16_uvw_curr_bias;
	u_i							=	((u_i) > 0 ? (u_i) : -(u_i) );
	/*����u���������ֵ�����ͱ���*/
	v_i							=	m_parg->ADC_DMA_buf[1] - m_parg->u16_uvw_curr_bias;
	v_i							=	((v_i) > 0 ? (v_i) : -(v_i) );
	/*����u���������ֵ�����ͱ���*/
	w_i							=	m_parg->ADC_DMA_buf[2] - m_parg->u16_uvw_curr_bias;
	w_i							=	((w_i) > 0 ? (w_i) : -(w_i) );	
	/*��������ƽ��*/
	real_i						=	(u_i + v_i + w_i)>>1;
	/*ѹ���˲���*/
	m_parg->u16_uvw_current		=	Single_Current_Average_X4_Filter(&real_i);
	
	if(m_sys_state.u8_cur_state == Run_state)	
	{
		if(m_parg->u16_uvw_current > 2000)
			m_parg->u16_uvw_current = 2000;												//���õ���ֵ����
	}
	
	m_motor_ctrl.u8_current_read_data_refreshed		=	1;								//��ȡ�����������ݸ���
}

	/*---------------------------------------------------------------------------
	��������			��Single_Current_Average_X4_Filter(uint16_t * new_data)
	��������			��	
							uint16_t * new_data	�»�ȡ����ָ��

	��������			��  ѡ����Ӧ��ͨ�����ݣ������databuf�н���ƽ��
	----------------------------------------------------------------------------*/

uint16_t	Single_Current_Average_X4_Filter(uint16_t * new_data)
{
//	int16_t		delta_value;															//����������һ�β������ݵĲ�ֵ
//	int16_t		pre_value;
	uint16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																//�����ԭʼ����ֵ���ܳ���4095
	
	if(m_motor_rt_para.uvw_buffer_used < 4)																			//bufferδ����
	{
		m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw]	=	*new_data;								//�������ݼ���buffer
		m_motor_rt_para.uvw_buffer_used++;
		m_motor_rt_para.filter_index_uvw 								=	m_motor_rt_para.uvw_buffer_used;			
		m_motor_rt_para.u16_uvw_sum										+=	*new_data;								//���������������
		return		m_motor_rt_para.u16_uvw_sum>>2;																	//����ƽ��ֵ
	}
	else																											//buffer��װ��
	{
		m_motor_rt_para.filter_index_uvw++;																			//����Ǩ��
		if(m_motor_rt_para.filter_index_uvw >= 4)
			m_motor_rt_para.filter_index_uvw =0;
		
		if(m_motor_rt_para.filter_index_uvw == 3)																	//�Ӵ��ڻ�����ֵ
			abandoned_value			=	m_motor_rt_para.history_data[0];
		else
			abandoned_value			=	m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw + 1];
		
//		if(m_motor_rt_para.filter_index_uvw == 0)															//��һ������
//			pre_value				=	(int16_t)m_motor_rt_para.history_data[3];
//		else
//			pre_value				=	(int16_t)m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw - 1];
//		
//		delta_value					=	(int16_t)*new_data - pre_value;												//���������������ϴ����ݵĲ�ֵ
//		
//		if((delta_value > NOISE_PULSATION_THRESHOLD) ||(delta_value < -NOISE_PULSATION_THRESHOLD))
//			m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw]	=	pre_value;		//��ֵ��ǰ�β���ֵ֮�������ֵ������
//		else
			m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw]	=	*new_data;
		
		m_motor_rt_para.u16_uvw_sum		+=	m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw];
		m_motor_rt_para.u16_uvw_sum		-=	abandoned_value;
		
		return	m_motor_rt_para.u16_uvw_sum>>2;
	}
	
}


	/*---------------------------------------------------------------------------
	��������			��Current_Average_X8_Filter(motor_runtime_para * m_parg)
	��������			��
	��������			������ֵ���д���ƽ���˲����˲����ڿ��Ϊ8
	----------------------------------------------------------------------------*/
void	Current_Average_X8_Filter(motor_runtime_para * m_parg)
{
	int16_t		u_i,v_i,w_i;
	uint16_t	real_i;
	/*����u���������ֵ�����ͱ���*/
	u_i							=	m_parg->ADC_DMA_buf[0] - m_parg->u16_uvw_curr_bias;	
	u_i							=	((u_i) > 0 ? (u_i) : -(u_i) );
	/*����v���������ֵ�����ͱ���*/
	v_i							=	m_parg->ADC_DMA_buf[1] - m_parg->u16_uvw_curr_bias;
	v_i							=	((v_i) > 0 ? (v_i) : -(v_i) );
	/*����w���������ֵ�����ͱ���*/
	w_i							=	m_parg->ADC_DMA_buf[2] - m_parg->u16_uvw_curr_bias;
	w_i							=	((w_i) > 0 ? (w_i) : -(w_i) );	
	/*��������ƽ��*/
	real_i						=	(u_i + v_i + w_i)>>1;
	/*ѹ���˲���*/
	m_parg->u16_uvw_current		=	Single_Current_Average_X8_Filter(&real_i);
	
	if(m_sys_state.u8_cur_state == Run_state)	
	{
		if(m_parg->u16_uvw_current > 2000)
			m_parg->u16_uvw_current = 2000;												//���õ���ֵ����
	}
	
	m_motor_ctrl.u8_current_read_data_refreshed		=	1;								//��ȡ�����������ݸ���
}


uint16_t	Single_Current_Average_X8_Filter(uint16_t * new_data)
{
	uint16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																//�����ԭʼ����ֵ���ܳ���4095
	
	if(m_motor_rt_para.uvw_buffer_used < 8)																			//bufferδ����
	{
		m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw]	=	*new_data;								//�������ݼ���buffer
		m_motor_rt_para.uvw_buffer_used++;
		m_motor_rt_para.filter_index_uvw 								=	m_motor_rt_para.uvw_buffer_used;			
		m_motor_rt_para.u16_uvw_sum										+=	*new_data;								//���������������
		return		m_motor_rt_para.u16_uvw_sum>>2;																	//����ƽ��ֵ
	}
	else																											//buffer��װ��
	{
		m_motor_rt_para.filter_index_uvw++;																			//����Ǩ��
		if(m_motor_rt_para.filter_index_uvw >= 8)
			m_motor_rt_para.filter_index_uvw =0;
		
		if(m_motor_rt_para.filter_index_uvw == 7)																	//�Ӵ��ڻ�����ֵ
			abandoned_value			=	m_motor_rt_para.history_data[0];
		else
			abandoned_value			=	m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw + 1];
		
		m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw]	=	*new_data;
		
		m_motor_rt_para.u16_uvw_sum		+=	m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw];
		m_motor_rt_para.u16_uvw_sum		-=	abandoned_value;
		
		return	m_motor_rt_para.u16_uvw_sum>>2;
	}
}







/*********************************** END OF FILE*****************************************/

