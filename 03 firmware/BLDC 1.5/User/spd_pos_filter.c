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

#include	"spd_pos_filter.h"
#include	"peripherial_init.h"
#include	"stm32f4xx.h"
#include	"stm32f4xx_tim.h"
#include	"stm32f4xx_dma.h"
#include	"global_parameters.h"
#include	"math.h"										//�������ʹ��ͷ�ļ�
#include	"arm_math.h"
#include	"jingle_math.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
typedef struct
{
	uint8_t	index;
	float	f_raw_data[2];
	float	f_acc;
}noise_buf_struct;


/* Private macro -------------------------------------------------------------*/
#define		T				0.001f
#define		T_sqr			0.000001f
#define		half_T_sqr		0.0000005f
#define		Q11				0.001f
#define		Q22				0.001f
#define		R11				0.001f
#define		R22				0.001f

/* Private variables ---------------------------------------------------------*/
//noise_buf_struct	nosie_buf;
KF_filter_struct	m_KF;				//�������˲����ṹ��


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	��������			��	Speed_Average_X4_Filter()
	��������			��	
	��������			��	������ֵ�˲���
	----------------------------------------------------------------------------*/
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
	��������			��	KF_Filter_Init()
	��������			��	
	��������			��	�������˲�����ʼ��
	----------------------------------------------------------------------------*/
void	KF_Filter_Init(KF_filter_struct * KF_struct)
{
	KF_struct->for_X[0]		=	0.0f;
	KF_struct->for_X[1]		=	0.0f;
	
	KF_struct->X_k[0]		=	0.0f;
	KF_struct->X_k[1]		=	0.0f;
	
	KF_struct->X_k_1[0]		=	0.0f;
	KF_struct->X_k_1[1]		=	0.0f;
	
	//���Э�������Ϊ�Խ����ڼ����л�ʵʱ����
	KF_struct->for_Pk[0]	=	0.1f;
	KF_struct->for_Pk[1]	=	0.0f;
	KF_struct->for_Pk[2]	=	0.0f;
	KF_struct->for_Pk[3]	=	0.1f;
	
	KF_struct->P_k_1[0]		=	0.1f;
	KF_struct->P_k_1[1]		=	0.0f;
	KF_struct->P_k_1[2]		=	0.0f;
	KF_struct->P_k_1[3]		=	0.1f;
	
	KF_struct->A[0]			=	1.0f;
	KF_struct->A[1]			=	T;
	KF_struct->A[2]			=	0.0f;
	KF_struct->A[3]			=	1.0f;
	
	KF_struct->B[0]			=	half_T_sqr;
	KF_struct->B[1]			=	T;
	
	//ϵͳ��������
	KF_struct->Q[0]			=	Q11;
	KF_struct->Q[1]			=	0.1f;
	KF_struct->Q[2]			=	0.1f;
	KF_struct->Q[3]			=	Q22;
	
	//ϵͳ��������
	KF_struct->R[0]			=	R11;
	KF_struct->R[1]			=	0.1f;
	KF_struct->R[2]			=	0.1f;
	KF_struct->R[3]			=	R22;
}

	/*---------------------------------------------------------------------------
	��������			��	KF_Filter()
	��������			��	
	��������			��	�������˲����㣬��Ҫ�Ƚ����˲�����ʼ��
	----------------------------------------------------------------------------*/
void	KF_Filter(int32_t * input_pos,float * input_spd, int32_t * output_pos, float * output_spd )
{
	float	PX[4];
	float	f_temp;
	
	/*��������ʱʹ���ٶȻ������*/
	m_KF.ctrl_a				=	m_motor_ctrl.m_motion_ctrl.f_set_speed;
	
	/*Xk��Ԥ��
		^X_k	=	A * X_k_1 + B * a_k	
	A:ϵͳ״̬����,		[1	T;	0	1]
	B:ϵͳ���ƾ���,		[half_T_sqr	;	T]
	*/
	m_KF.for_X[0]			=	m_KF.X_k_1[0] + T * m_KF.X_k_1[1] + half_T_sqr * m_KF.ctrl_a;				//�Ƕȵ�Ԥ����
	m_KF.for_X[1]			=	m_KF.X_k_1[1] + T * m_KF.ctrl_a;											//ת�ٵ�Ԥ����
	
	/*Ԥ������Э�������
		^P_k	=	A * P_k_1 * A_T + Q 
		(�� B * Q * B_T ��Q���)
	*/
	Matrix_Mult_Matrix	((volatile float *)&(m_KF.A),  	2, 2,
						 (volatile float *)&(m_KF.P_k_1),  2, 2, PX);										//PX 	= A * P_k_1
	
	Matrix_Mult_MatrixT	(PX, 2, 2,
						 (volatile float *)&(m_KF.A),  2, 2, (volatile float *)&(m_KF.P_k_1));				//P_k_1	= PX * A'= A * P_k_1 * A'
	
	Matrix_Plus_Matrix	((volatile float *)&(m_KF.P_k_1),  2, 2,
						 (volatile float *)&(m_KF.Q),  2, 2,  (volatile float *)&(m_KF.for_Pk));			//Pk 	= P_k_1 + Q = A * P * A' + Q
	
	/*���㿨��������
		Kk		=	for_Pk * C /(C * for_Pk * C_T + R)
	C:ϵͳ��������,		[1	0]
	*/
	m_KF.Kk[0]				=	m_KF.for_Pk[0] / (m_KF.for_Pk[0] + m_KF.R[0]);
	m_KF.Kk[1]				=	m_KF.for_Pk[2] / (m_KF.for_Pk[2] + m_KF.R[1]);
	
	/*�������Xk�����µ�Xk_1��
		Xk		=	for_Xk + Kk * [Zk - C * for_Xk]		
	Zk:ϵͳ��ǰ������,�����ٶ�
	*/
	f_temp					=	*input_spd	-	m_KF.for_X[0];
	m_KF.X_k_1[0]				=	m_KF.for_X[0] + m_KF.Kk[0] * f_temp;
	m_KF.X_k_1[1]				=	m_KF.for_X[1] + m_KF.Kk[1] * f_temp;
	
	/*���Э���������£����µ�P_k_1��
		Pk_1		=	[I - Kk * C] * for_Pk
	*/
	PX[0]					=	1.0f - m_KF.Kk[0];
	PX[1]					=	0.0f;
	PX[2]					=	- m_KF.Kk[1];
	PX[3]					=	1.0f;
	Matrix_Plus_Matrix	(PX,  2, 2,
						 (volatile float *)&(m_KF.for_Pk),  2, 2,  (volatile float *)&(m_KF.P_k_1));
						 
	* output_pos			=	(int32_t)m_KF.X_k_1[0];
	* output_spd			=	m_KF.X_k_1[1];
}



	/*---------------------------------------------------------------------------
	��������			��Read_IncEncoder(int32_t * encoder_num)
	��������			��
	��������			��	��ȡ�������������ݣ����µ�ȫ�ֱ�����
	----------------------------------------------------------------------------*/
void	Read_IncEncoder(void)
{
	uint16_t		temp_delta;
	uint32_t	f_width		=	0;
	float		f_temp		=	0.0f;
	float		f_velocity	=	0.0f;
	
	
	
	m_motor_rt_para.m_inc_encoder.u16_encoder_last_read		=	m_motor_rt_para.m_inc_encoder.u16_encoder_curr_read;						//��ȡ���±���������
	m_motor_rt_para.m_inc_encoder.u16_encoder_curr_read		=	TIM3->CNT;
	
	temp_delta	=	(m_motor_rt_para.m_inc_encoder.u16_encoder_curr_read - m_motor_rt_para.m_inc_encoder.u16_encoder_last_read);
	
	m_motor_rt_para.m_inc_encoder.i32_pulse_cnt				+=	(int32_t)temp_delta;																//��ȡ��ǰλ����Ϣ
	
	/*����temp_dalta�ж�ת�ٷ���*/
	if(temp_delta < 0)
		m_motor_rt_para.m_inc_encoder.u8_velocity_sign		=	0;
	else
		m_motor_rt_para.m_inc_encoder.u8_velocity_sign		=	1;
	
	/*�������ֵ*/
	f_width		=	m_motor_rt_para.m_inc_encoder.u32_pulse_width_buf[1];// + m_motor_rt_para.m_encoder.u32_pulse_width_buf[2] + m_motor_rt_para.m_encoder.u32_pulse_width_buf[3] +m_motor_rt_para.m_encoder.u32_pulse_width_buf[4];
//	f_width		=	f_width>>2;
	
	/*M/T���Զ�ת�� f_shaft_cal_speed*/
	if(m_motor_rt_para.m_inc_encoder.u8_M_or_T == M_METHORD)
	{
		/*ʹ��M��ʱ���������ٶȲ�������12.5rpsʱ������T��*/
		if(((m_motor_rt_para.m_inc_encoder.f_motor_cal_speed < 12.5f) && (m_motor_rt_para.m_inc_encoder.f_motor_cal_speed > -12.5f)) && (f_width != 0))
		{
			/*T�����٣����Ǽ�ʱ*/
			// ������4096�ߣ��ı�Ƶ��Ϊ16384 Hz��T������ʱ������������أ�
			// ��ȡ�������� N����һ����������ʱ��	t = N / 42000000 ��
			// ����תһȦ��ʱ��Ϊ�� t2 = t*16384 / 2 
			// ����ת��Ϊ��1/t2 =	5126.953125 / N
			f_velocity										=	5126.953125f/((float)f_width);
			m_motor_rt_para.m_inc_encoder.u8_M_or_T			=	T_METHORD;
		}
		else
		{
			f_temp											=	(float)temp_delta;		
			/*M�����٣���ʱ���*/
			f_velocity										=	f_temp / 16.384f;				
		}
	}
	else
	{
		/*ʹ��T��ʱ���������ٶȲ�������12.5rpsʱ������M��*/
		if(((m_motor_rt_para.m_inc_encoder.f_motor_cal_speed > 12.5f) && (m_motor_rt_para.m_inc_encoder.f_motor_cal_speed < -12.5f)) || (f_width == 0) )
		{
			f_temp		=	(float)temp_delta;		
			/*M�����٣���ʱ���*/
			f_velocity										=	f_temp / 16.384f;
			m_motor_rt_para.m_inc_encoder.u8_M_or_T			=	M_METHORD;
		}
		/*T�����٣����Ǽ�ʱ*/
		else
		{			
			f_velocity										=	5126.953125f/((float)f_width);
		}
	}

	//���ݷ����־λ�����ٶ�����
	arm_abs_f32(&f_velocity,&(m_motor_rt_para.m_inc_encoder.f_motor_cal_speed),1);
	
	if(m_motor_rt_para.m_inc_encoder.u8_velocity_sign == 0)
		m_motor_rt_para.m_inc_encoder.f_motor_cal_speed		=	-m_motor_rt_para.m_inc_encoder.f_motor_cal_speed;
	else
		m_motor_rt_para.m_inc_encoder.f_motor_cal_speed		=	m_motor_rt_para.m_inc_encoder.f_motor_cal_speed;
	
//	/*��Ұֵ*/
//	m_motor_rt_para.m_encoder.f_motor_cal_speed				=	Noise_killer(&nosie_buf,m_motor_rt_para.m_encoder.f_motor_cal_speed);
	
	/*��λ��ת�ٽ��п������˲�*/
	
	
	m_motor_ctrl.m_motion_ctrl.u8_speed_read_data_refreshed				=	1;
}



	/*---------------------------------------------------------------------------
	��������			��Requir_AbsEncoder(void)
	��������			��
	��������			��	��ȡ����ֵ���������ݣ����µ�ȫ�ֱ�����
							����spi��dmaͨ��������ֵ�������Ľӿ�ΪSSI����Ҫ�ȴ�
							100us���Ҳ��ܽ��ܵ�����
	----------------------------------------------------------------------------*/
void	Requir_AbsEncoder(void)
{
	SPI_DMA_ReadData(4);
}


	/*---------------------------------------------------------------------------
	��������			��	Read_AbsEncoder(int32_t * encoder_num)
	��������			��
	��������			��	����ȡ������д��encoder_num
							����m_motor_rt_para.m_encoder.u8_abs_raw_data
	----------------------------------------------------------------------------*/
void	Read_AbsEncoder(void)
{
	uint16_t	data_high,data_low ;
	uint16_t	data_1,data_2,data_3;
	int16_t		temp[2];
	
	data_low	=	(uint16_t)(m_motor_rt_para.m_abs_encoder.u8_abs_raw_data[2] << 8) || (m_motor_rt_para.m_abs_encoder.u8_abs_raw_data[3]);
	
	/*�鿴��9λ����λ�Ƿ��б���*/
	if(data_low & 0x01FF)																//��9λ�о�������
	{
		m_motor_rt_para.m_abs_encoder.u8_abs_data_refreshed		=	0;
		return;
	}
	else																				//��9λ�޾�������
	{
		data_1		=	(uint16_t)m_motor_rt_para.m_abs_encoder.u8_abs_raw_data[0];
		data_2		=	(uint16_t)m_motor_rt_para.m_abs_encoder.u8_abs_raw_data[1];
		data_3		=	(uint16_t)m_motor_rt_para.m_abs_encoder.u8_abs_raw_data[2];
		
		data_1		=	data_1 << 9;
		data_2		=	data_2 << 1;
		data_3		=	data_3 >> 7;

		m_motor_rt_para.m_abs_encoder.u16_abs_cnt				=	data_1 | data_2 | data_3;
		
		m_motor_rt_para.m_abs_encoder.u16_abs_cnt				=	Noise_Killer(	m_motor_rt_para.m_abs_encoder.history_data,
																					m_motor_rt_para.m_abs_encoder.u16_abs_cnt,
																					temp);
		
		m_motor_rt_para.m_abs_encoder.f_abs_pos					=	(float)(m_motor_rt_para.m_abs_encoder.u16_abs_cnt) / 182.0416f;		//��ǰ���ٶ˽Ƕ�
		m_motor_rt_para.m_abs_encoder.u8_abs_data_refreshed		=	2;
		
		if(m_motor_rt_para.m_abs_encoder.u8_abs_pos_init_flag == 0)
		{
			m_motor_rt_para.m_abs_encoder.f_abs_pos_init		=	m_motor_rt_para.m_abs_encoder.f_abs_pos;
			m_motor_rt_para.m_abs_encoder.u8_abs_pos_init_flag	=	1;
		}
		
		return;
	}
}


	/*---------------------------------------------------------------------------
	��������			��	Noise_killer(void)
	��������			��	buf			�洢��ʷ���ݵ�buf
							curr_value	��ǰ������ֵ
	��������			��	�˳�Ұֵ
							���س�Ұֵ�������
							������ݱ仯�����ƣ��������������ݵĲ�ֵ������ݺϷ���
							buf���ȴ���3
							temp���ȴ���2
	----------------------------------------------------------------------------*/
uint16_t	Noise_Killer(uint16_t	* buf, uint16_t curr_value,int16_t	*temp)
{	
	if(m_motor_rt_para.m_abs_encoder.u8_abs_history_buf_full > 3)
	{
		temp[1]	=	curr_value - buf[2];						//��ǰ������
		temp[2]	=	buf[2] - buf[1];							//������һ������
		
		arm_abs_q15(&temp[1],&temp[1],1);						//�����������ֵ
		arm_abs_q15(&temp[2],&temp[2],1);
		
		if((temp[1] > (temp[2] * 4)) &&(temp[1] > 100))			//����������һ��������4�������Ҵ�����ֵ����Ϊ��Ұֵ
		{
			buf[0]	=	buf[1];
			buf[1]	=	buf[2];
			buf[2]	=	buf[2] + buf[2] - buf[1];
			return buf[2];
		}
		else
		{
			buf[0]	=	buf[1];
			buf[1]	=	buf[2];
			buf[2]	=	curr_value;
			return buf[2];
		}
	}
	else
	{
		buf[m_motor_rt_para.m_abs_encoder.u8_abs_history_buf_full]	=	curr_value;
		m_motor_rt_para.m_abs_encoder.u8_abs_history_buf_full++;
		return	curr_value;
	}
}





/*****************************END OF FILE************************************/

