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

//�������˲��������ṹ��
typedef	struct
{
	float	for_X[2];					//���ٶ�λ�õ�Ԥ����,X1Ϊ�Ƕ�Ԥ��ֵ��X2Ϊ�ٶ�Ԥ��ֵ
	float	X_k[2];						//��ǰ�ļ�����
	float	X_k_1[2];					//ǰһ�ļ�����	
	
	float	for_Pk[4];					//kʱ������Э����Ԥ��ֵ
	float	P_k_1[4];					//k-1ʱ�̼�������Э�������	
	float	ctrl_a;						//�����������ٶ�
	float	Kk;							//kʱ�̼���Ŀ���������
	float	Q[4];						//ϵͳ��������������
	float	R[4];						//ϵͳ��������
}KF_filter_struct;
/* Private macro -------------------------------------------------------------*/
#define		T		0.001f
#define		T2		0.000001f
#define		Q11		0.001f;
#define		Q22		0.001f;
#define		R11		0.001f;
#define		R22		0.001f;


/* Private variables ---------------------------------------------------------*/
//noise_buf_struct	nosie_buf;
KF_filter_struct	m_KF;
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
	��������			��	Noise_killer()
	��������			��	noise_buf_struct * noise_buf
							float new_data
	��������			��	�˳��ٶ�ֵ�е�Ұֵ
							�����������ε��ٶȱ仯����
							����һ��Ұֵ������������һ�ĵ����ݼ�������ٶȲ���
							Ч������
	----------------------------------------------------------------------------*/
float	Noise_killer(noise_buf_struct * noise_buf, float new_data)
{
	float	f_temp_acc;
	float	f_acc_acc;	
	float	f_temp_acc_acc_abs;

	/*��ǰ�ļ��ٶ�*/
	f_temp_acc								=	new_data - noise_buf->f_raw_data[1];
	/*�������μ��ٶ�ֵ��ľ���ֵ*/
	f_acc_acc								=	f_temp_acc	-	noise_buf->f_acc;
	arm_abs_f32(&f_acc_acc,&f_temp_acc_acc_abs,1);
	
	if(f_temp_acc_acc_abs > 1.0f)					//��ǰ���ٶ���������������ǰһ���ٶ�����������Ұֵ������acc_absΪ0�����
	{
		noise_buf->f_raw_data[0]				=	noise_buf->f_raw_data[1];
		noise_buf->f_raw_data[1] 				=	noise_buf->f_raw_data[1] + noise_buf->f_acc;
	}
	else
	{
		noise_buf->f_raw_data[0]				=	noise_buf->f_raw_data[1];
		noise_buf->f_raw_data[1]				=	new_data;
		noise_buf->f_acc						=	f_temp_acc;
	}
	return	noise_buf->f_raw_data[1];	

}


	/*---------------------------------------------------------------------------
	��������			��	KF_Filter()
	��������			��	
	��������			��	�������˲����㣬��Ҫ�Ƚ����˲�����ʼ��
	----------------------------------------------------------------------------*/
void	KF_Filter(int32_t * input_pos,float * input_spd, int32_t * output_pos, float * output_spd )
{
	
}

	/*---------------------------------------------------------------------------
	��������			��Read_IncEncoder(int32_t * encoder_num)
	��������			��
	��������			��	��ȡ�������������ݣ����µ�ȫ�ֱ�����
	----------------------------------------------------------------------------*/
void	Read_IncEncoder(void)
{
	int32_t		temp_delta;
	uint32_t	f_width		=	0;
	float		f_temp		=	0.0f;
	float		f_velocity	=	0.0f;
	
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
//	TIM_DMACmd(TIM5,TIM_DMA_CC1,DISABLE);
	
	/*�������ֵ*/
	f_width		=	m_motor_rt_para.m_encoder.u32_pulse_width_buf[1];// + m_motor_rt_para.m_encoder.u32_pulse_width_buf[2] + m_motor_rt_para.m_encoder.u32_pulse_width_buf[3] +m_motor_rt_para.m_encoder.u32_pulse_width_buf[4];
//	f_width		=	f_width>>2;
	
	/*M/T���Զ�ת�� f_shaft_cal_speed*/
	if(m_motor_rt_para.m_encoder.u8_M_or_T == M_METHORD)
	{
		/*ʹ��M��ʱ���������ٶȲ�������12.5rpsʱ������T��*/
		if(((m_motor_rt_para.m_encoder.f_motor_cal_speed < 12.5f) && (m_motor_rt_para.m_encoder.f_motor_cal_speed > -12.5f)) && (f_width != 0))
		{
			/*T�����٣����Ǽ�ʱ*/
			// ������4096�ߣ��ı�Ƶ��Ϊ16384 Hz��T������ʱ������������أ�
			// ��ȡ�������� N����һ����������ʱ��	t = N / 42000000 ��
			// ����תһȦ��ʱ��Ϊ�� t2 = t*16384 / 2 
			// ����ת��Ϊ��1/t2 =	5126.953125 / N
			f_velocity										=	5126.953125f/((float)f_width);
			m_motor_rt_para.m_encoder.u8_M_or_T				=	T_METHORD;
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
		if(((m_motor_rt_para.m_encoder.f_motor_cal_speed > 12.5f) && (m_motor_rt_para.m_encoder.f_motor_cal_speed < -12.5f)) || (f_width == 0) )
		{
			f_temp		=	(float)temp_delta;		
			/*M�����٣���ʱ���*/
			f_velocity										=	f_temp / 16.384f;
			m_motor_rt_para.m_encoder.u8_M_or_T				=	M_METHORD;
		}
		/*T�����٣����Ǽ�ʱ*/
		else
		{			
			f_velocity										=	5126.953125f/((float)f_width);
		}
	}
	/*����DMAͨ��*/
//	TIM_DMACmd(TIM5,TIM_DMA_CC1,ENABLE);
	
	//���ݷ����־λ�����ٶ�����
	arm_abs_f32(&f_velocity,&(m_motor_rt_para.m_encoder.f_motor_cal_speed),1);
	
	if(m_motor_rt_para.m_encoder.u8_velocity_sign == 0)
		m_motor_rt_para.m_encoder.f_motor_cal_speed			=	-m_motor_rt_para.m_encoder.f_motor_cal_speed;
	else
		m_motor_rt_para.m_encoder.f_motor_cal_speed			=	m_motor_rt_para.m_encoder.f_motor_cal_speed;
	
	/*��Ұֵ*/
//	m_motor_rt_para.m_encoder.f_motor_cal_speed				=	Noise_killer(&nosie_buf,m_motor_rt_para.m_encoder.f_motor_cal_speed);
	
	m_motor_ctrl.u8_speed_read_data_refreshed				=	1;
}








/*****************************END OF FILE************************************/

