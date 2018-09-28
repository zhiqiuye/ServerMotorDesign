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


/*	��������ʱ���������������ɼ��ĵ���ֵ����ʵ����ֵ��ϵ
-------------------------------------------------------------------------------
	��תʱ��
			Hall State			Phase A		Phase B		Phase C
				1					0			-I			I
				2					-I			I			0
				3					-I			0			I
				4					I			0			-I
				5					I			-I			0
				6					0			I			-I

	��תʱ��
			Hall State			Phase A		Phase B		Phase C
				1					0			I			-I
				2					I			-I			0
				3					I			0			-I
				4					-I			0			I
				5					-I			I			0
				6					0			-I			I
--------------------------------------------------------------------------------
*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
//�����˲��� ��ֵ
#define		NOISE_PULSATION_THRESHOLD					50



/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void		Single_Current_Average_X8_Filter_6Step(int16_t * new_data);
void		Single_Current_Average_X8_Filter_SVPWM(int16_t * u_data,int16_t * v_data,int16_t * w_data);



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
	��������			��	Current_Average_X8_Filter(motor_runtime_para * m_parg)
	��������			��
	��������			��	����ֵ���д���ƽ���˲����˲����ڿ��Ϊ8
						����ֵ����ƽ��ֵ��ֵ������
						�����˲�ǰ�����ݴ����m_motor_rt_para.ADC_DMA_buf��
						ADC_DMA_buf 0-2 ��Ӧ UVW
						��·���ݼ����ʱ0.8us��pwm����50us
	----------------------------------------------------------------------------*/
void	Current_Average_X8_Filter(current_sensor_state * m_parg)
{
	int16_t		real_i;
	int16_t		real_u,real_v,real_w;
	/*��������ʱ�������˲�*/
	if(m_motor_ctrl.m_sys_state.u8_use_svpwm	==	0)
	{
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
		Single_Current_Average_X8_Filter_6Step(&real_i);
		
		if(m_motor_ctrl.m_sys_state.u8_cur_state == Run_state)	
		{
			if(m_parg->i16_uvw_current > 2000)
				m_parg->i16_uvw_current = 2000;																		//���õ���ֵ����
		}	
	}
	/*SVPWMʱ�ĵ����˲�*/
	else
	{
		real_u	=	m_parg->adc_dma_shadow[0] - (int16_t)m_parg->i16_uvw_curr_bias;
		real_v	=	m_parg->adc_dma_shadow[1] - (int16_t)m_parg->i16_uvw_curr_bias;
		real_w	=	m_parg->adc_dma_shadow[2] - (int16_t)m_parg->i16_uvw_curr_bias;
		Single_Current_Average_X8_Filter_SVPWM(&real_u,&real_v,&real_w);
	}
}


void	Single_Current_Average_X8_Filter_6Step(int16_t * new_data)
{
	int16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																						//�����ԭʼ����ֵ���ܳ���4095
	
	if(m_motor_rt_para.m_current_filter.uvw_buffer_used < 8)													//bufferδ����
	{
		m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*new_data;								//�������ݼ���buffer
		m_motor_rt_para.m_current_filter.uvw_buffer_used++;
		m_motor_rt_para.m_current_filter.filter_index_uvw 								=	m_motor_rt_para.m_current_filter.uvw_buffer_used;			
		m_motor_rt_para.m_current_filter.i16_uvw_sum									+=	*new_data;			//���������������
		m_motor_rt_para.m_current_sensor.i16_uvw_current	=	m_motor_rt_para.m_current_filter.i16_uvw_sum / m_motor_rt_para.m_current_filter.uvw_buffer_used;	//����ƽ��ֵ
	}
	else																										//buffer��װ��
	{
		m_motor_rt_para.m_current_filter.filter_index_uvw++;													//����Ǩ��
		if(m_motor_rt_para.m_current_filter.filter_index_uvw >= 8)
			m_motor_rt_para.m_current_filter.filter_index_uvw =0;
		
		if(m_motor_rt_para.m_current_filter.filter_index_uvw == 7)												//�Ӵ��ڻ�����ֵ
			abandoned_value			=	m_motor_rt_para.m_current_filter.history_data[0][0];
		else
			abandoned_value			=	m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw + 1];
		
		/*��Ұֵ*/
		if(((*new_data - m_motor_rt_para.m_current_sensor.i16_uvw_current) > NOISE_PULSATION_THRESHOLD)
			||((*new_data - m_motor_rt_para.m_current_sensor.i16_uvw_current) < - NOISE_PULSATION_THRESHOLD))
			m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	m_motor_rt_para.m_current_sensor.i16_uvw_current;
		else		
			m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*new_data;
		
		m_motor_rt_para.m_current_filter.i16_uvw_sum		+=	m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw];
		m_motor_rt_para.m_current_filter.i16_uvw_sum		-=	abandoned_value;
		
		m_motor_rt_para.m_current_sensor.i16_uvw_current	=	m_motor_rt_para.m_current_filter.i16_uvw_sum>>3;
	}
}



void	Single_Current_Average_X8_Filter_SVPWM(int16_t * u_data,int16_t * v_data,int16_t * w_data)
{
	int16_t	abandoned_value[3];
	
	if(*u_data > 4095)	*u_data = 4095;																			//�����ԭʼ����ֵ���ܳ���4095
	if(*v_data > 4095)	*v_data = 4095;																			//�����ԭʼ����ֵ���ܳ���4095
	if(*w_data > 4095)	*w_data = 4095;																			//�����ԭʼ����ֵ���ܳ���4095
	
	if(m_motor_rt_para.m_current_filter.uvw_buffer_used < 8)													//bufferδ����
	{
		m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*u_data;								//�������ݼ���buffer
		m_motor_rt_para.m_current_filter.history_data[1][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*v_data;	
		m_motor_rt_para.m_current_filter.history_data[2][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*w_data;	
		m_motor_rt_para.m_current_filter.uvw_buffer_used++;
		m_motor_rt_para.m_current_filter.filter_index_uvw 								=	m_motor_rt_para.m_current_filter.uvw_buffer_used;			
		m_motor_rt_para.m_current_filter.i16_u_sum										+=	*u_data;			//���������������
		m_motor_rt_para.m_current_filter.i16_v_sum										+=	*v_data;			//���������������
		m_motor_rt_para.m_current_filter.i16_w_sum										+=	*w_data;			//���������������
		
		m_motor_rt_para.m_current_sensor.i16_u_current	=	m_motor_rt_para.m_current_filter.i16_u_sum / m_motor_rt_para.m_current_filter.uvw_buffer_used;
		m_motor_rt_para.m_current_sensor.i16_v_current	=	m_motor_rt_para.m_current_filter.i16_v_sum / m_motor_rt_para.m_current_filter.uvw_buffer_used;
		m_motor_rt_para.m_current_sensor.i16_w_current	=	m_motor_rt_para.m_current_filter.i16_w_sum / m_motor_rt_para.m_current_filter.uvw_buffer_used;
	}
	else																										//buffer��װ��
	{
		m_motor_rt_para.m_current_filter.filter_index_uvw++;													//����Ǩ��
		if(m_motor_rt_para.m_current_filter.filter_index_uvw >= 8)
			m_motor_rt_para.m_current_filter.filter_index_uvw =0;
		
		if(m_motor_rt_para.m_current_filter.filter_index_uvw == 7)												//�Ӵ��ڻ�����ֵ
		{
			abandoned_value[0]			=	m_motor_rt_para.m_current_filter.history_data[0][0];
			abandoned_value[1]			=	m_motor_rt_para.m_current_filter.history_data[1][0];
			abandoned_value[2]			=	m_motor_rt_para.m_current_filter.history_data[2][0];
		}
		else
		{
			abandoned_value[0]			=	m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw + 1];
			abandoned_value[1]			=	m_motor_rt_para.m_current_filter.history_data[1][m_motor_rt_para.m_current_filter.filter_index_uvw + 1];
			abandoned_value[2]			=	m_motor_rt_para.m_current_filter.history_data[2][m_motor_rt_para.m_current_filter.filter_index_uvw + 1];
		}
		/*��Ұֵ*/
		if(((*u_data - m_motor_rt_para.m_current_sensor.i16_uvw_current) > NOISE_PULSATION_THRESHOLD)
			||((*u_data - m_motor_rt_para.m_current_sensor.i16_uvw_current) < - NOISE_PULSATION_THRESHOLD))
			m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	m_motor_rt_para.m_current_sensor.i16_u_current;
		else		
			m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*u_data;
		
		if(((*v_data - m_motor_rt_para.m_current_sensor.i16_uvw_current) > NOISE_PULSATION_THRESHOLD)
			||((*v_data - m_motor_rt_para.m_current_sensor.i16_uvw_current) < - NOISE_PULSATION_THRESHOLD))
			m_motor_rt_para.m_current_filter.history_data[1][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	m_motor_rt_para.m_current_sensor.i16_v_current;
		else		
			m_motor_rt_para.m_current_filter.history_data[1][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*v_data;		

		if(((*w_data - m_motor_rt_para.m_current_sensor.i16_uvw_current) > NOISE_PULSATION_THRESHOLD)
			||((*w_data - m_motor_rt_para.m_current_sensor.i16_uvw_current) < - NOISE_PULSATION_THRESHOLD))
			m_motor_rt_para.m_current_filter.history_data[2][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	m_motor_rt_para.m_current_sensor.i16_w_current;
		else		
			m_motor_rt_para.m_current_filter.history_data[2][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*w_data;		
		
		m_motor_rt_para.m_current_filter.i16_u_sum		+=	m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw];
		m_motor_rt_para.m_current_filter.i16_u_sum		-=	abandoned_value[0];
		
		m_motor_rt_para.m_current_filter.i16_v_sum		+=	m_motor_rt_para.m_current_filter.history_data[1][m_motor_rt_para.m_current_filter.filter_index_uvw];
		m_motor_rt_para.m_current_filter.i16_v_sum		-=	abandoned_value[1];

		m_motor_rt_para.m_current_filter.i16_w_sum		+=	m_motor_rt_para.m_current_filter.history_data[2][m_motor_rt_para.m_current_filter.filter_index_uvw];
		m_motor_rt_para.m_current_filter.i16_w_sum		-=	abandoned_value[2];		
		
		/*����������ֵ*/
		m_motor_rt_para.m_current_sensor.i16_u_current	=	m_motor_rt_para.m_current_filter.i16_u_sum>>3;
		m_motor_rt_para.m_current_sensor.i16_v_current	=	m_motor_rt_para.m_current_filter.i16_v_sum>>3;
		m_motor_rt_para.m_current_sensor.i16_w_current	=	m_motor_rt_para.m_current_filter.i16_w_sum>>3;
	}
}




/*********************************** END OF FILE*****************************************/

