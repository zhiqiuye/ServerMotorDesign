/**
  ******************************************************************************
  * @file    Project/user/spd_pos_filter.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20180814
  * @brief   编码器数据读取，速度位置解算
  ******************************************************************************
  ******************************************************************************
  */

#include	"spd_pos_filter.h"
#include	"peripherial_init.h"
#include	"stm32f4xx.h"
#include	"stm32f4xx_tim.h"
#include	"stm32f4xx_dma.h"
#include	"global_parameters.h"
#include	"math.h"										//浮点计算使用头文件
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
KF_filter_struct	m_KF;				//卡尔曼滤波器结构体


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	函数名称			：	Speed_Average_X4_Filter()
	参数含义			：	
	函数功能			：	滑动均值滤波器
	----------------------------------------------------------------------------*/
float	Speed_Average_X4_Filter(int32_t * new_data, int32_t * buffer, uint8_t * buf_used, int32_t * sum)
{
	float result = 0;

	if(*buf_used < 4)										//均值滤波器未填满
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
	函数名称			：	KF_Filter_Init()
	参数含义			：	
	函数功能			：	卡尔曼滤波器初始化
	----------------------------------------------------------------------------*/
void	KF_Filter_Init(KF_filter_struct * KF_struct)
{
	KF_struct->for_X[0]		=	0.0f;
	KF_struct->for_X[1]		=	0.0f;
	
	KF_struct->X_k[0]		=	0.0f;
	KF_struct->X_k[1]		=	0.0f;
	
	KF_struct->X_k_1[0]		=	0.0f;
	KF_struct->X_k_1[1]		=	0.0f;
	
	//误差协方差矩阵为对角阵，在计算中会实时更新
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
	
	//系统过程噪声
	KF_struct->Q[0]			=	Q11;
	KF_struct->Q[1]			=	0.1f;
	KF_struct->Q[2]			=	0.1f;
	KF_struct->Q[3]			=	Q22;
	
	//系统测量噪声
	KF_struct->R[0]			=	R11;
	KF_struct->R[1]			=	0.1f;
	KF_struct->R[2]			=	0.1f;
	KF_struct->R[3]			=	R22;
}

	/*---------------------------------------------------------------------------
	函数名称			：	KF_Filter()
	参数含义			：	
	函数功能			：	卡尔曼滤波计算，需要先进行滤波器初始化
	----------------------------------------------------------------------------*/
void	KF_Filter(int32_t * input_pos,float * input_spd, int32_t * output_pos, float * output_spd )
{
	float	PX[4];
	float	f_temp;
	
	/*控制量暂时使用速度环的输出*/
	m_KF.ctrl_a				=	m_motor_ctrl.m_motion_ctrl.f_set_speed;
	
	/*Xk的预测
		^X_k	=	A * X_k_1 + B * a_k	
	A:系统状态矩阵,		[1	T;	0	1]
	B:系统控制矩阵,		[half_T_sqr	;	T]
	*/
	m_KF.for_X[0]			=	m_KF.X_k_1[0] + T * m_KF.X_k_1[1] + half_T_sqr * m_KF.ctrl_a;				//角度的预测量
	m_KF.for_X[1]			=	m_KF.X_k_1[1] + T * m_KF.ctrl_a;											//转速的预测量
	
	/*预测误差的协方差矩阵
		^P_k	=	A * P_k_1 * A_T + Q 
		(将 B * Q * B_T 用Q替代)
	*/
	Matrix_Mult_Matrix	((volatile float *)&(m_KF.A),  	2, 2,
						 (volatile float *)&(m_KF.P_k_1),  2, 2, PX);										//PX 	= A * P_k_1
	
	Matrix_Mult_MatrixT	(PX, 2, 2,
						 (volatile float *)&(m_KF.A),  2, 2, (volatile float *)&(m_KF.P_k_1));				//P_k_1	= PX * A'= A * P_k_1 * A'
	
	Matrix_Plus_Matrix	((volatile float *)&(m_KF.P_k_1),  2, 2,
						 (volatile float *)&(m_KF.Q),  2, 2,  (volatile float *)&(m_KF.for_Pk));			//Pk 	= P_k_1 + Q = A * P * A' + Q
	
	/*计算卡尔曼增益
		Kk		=	for_Pk * C /(C * for_Pk * C_T + R)
	C:系统测量矩阵,		[1	0]
	*/
	m_KF.Kk[0]				=	m_KF.for_Pk[0] / (m_KF.for_Pk[0] + m_KF.R[0]);
	m_KF.Kk[1]				=	m_KF.for_Pk[2] / (m_KF.for_Pk[2] + m_KF.R[1]);
	
	/*计算输出Xk，更新到Xk_1中
		Xk		=	for_Xk + Kk * [Zk - C * for_Xk]		
	Zk:系统当前测量量,即角速度
	*/
	f_temp					=	*input_spd	-	m_KF.for_X[0];
	m_KF.X_k_1[0]				=	m_KF.for_X[0] + m_KF.Kk[0] * f_temp;
	m_KF.X_k_1[1]				=	m_KF.for_X[1] + m_KF.Kk[1] * f_temp;
	
	/*误差协方差矩阵更新，更新到P_k_1中
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
	函数名称			：Read_IncEncoder(int32_t * encoder_num)
	参数含义			：
	函数功能			：	获取增量编码器数据，更新到全局变量中
	----------------------------------------------------------------------------*/
void	Read_IncEncoder(void)
{
	uint16_t		temp_delta;
	uint32_t	f_width		=	0;
	float		f_temp		=	0.0f;
	float		f_velocity	=	0.0f;
	
	
	
	m_motor_rt_para.m_inc_encoder.u16_encoder_last_read		=	m_motor_rt_para.m_inc_encoder.u16_encoder_curr_read;						//获取最新编码器读数
	m_motor_rt_para.m_inc_encoder.u16_encoder_curr_read		=	TIM3->CNT;
	
	temp_delta	=	(m_motor_rt_para.m_inc_encoder.u16_encoder_curr_read - m_motor_rt_para.m_inc_encoder.u16_encoder_last_read);
	
	m_motor_rt_para.m_inc_encoder.i32_pulse_cnt				+=	(int32_t)temp_delta;																//获取当前位置信息
	
	/*根据temp_dalta判断转速方向*/
	if(temp_delta < 0)
		m_motor_rt_para.m_inc_encoder.u8_velocity_sign		=	0;
	else
		m_motor_rt_para.m_inc_encoder.u8_velocity_sign		=	1;
	
	/*对脉宽均值*/
	f_width		=	m_motor_rt_para.m_inc_encoder.u32_pulse_width_buf[1];// + m_motor_rt_para.m_encoder.u32_pulse_width_buf[2] + m_motor_rt_para.m_encoder.u32_pulse_width_buf[3] +m_motor_rt_para.m_encoder.u32_pulse_width_buf[4];
//	f_width		=	f_width>>2;
	
	/*M/T法自动转换 f_shaft_cal_speed*/
	if(m_motor_rt_para.m_inc_encoder.u8_M_or_T == M_METHORD)
	{
		/*使用M法时，如果电机速度测量低于12.5rps时，改用T法*/
		if(((m_motor_rt_para.m_inc_encoder.f_motor_cal_speed < 12.5f) && (m_motor_rt_para.m_inc_encoder.f_motor_cal_speed > -12.5f)) && (f_width != 0))
		{
			/*T法测速，定角计时*/
			// 编码器4096线，四倍频后为16384 Hz，T法测速时检测两个上升沿，
			// 获取脉冲周期 N，则一个脉冲周期时间	t = N / 42000000 秒
			// 码盘转一圈的时间为： t2 = t*16384 / 2 
			// 码盘转速为：1/t2 =	5126.953125 / N
			f_velocity										=	5126.953125f/((float)f_width);
			m_motor_rt_para.m_inc_encoder.u8_M_or_T			=	T_METHORD;
		}
		else
		{
			f_temp											=	(float)temp_delta;		
			/*M法测速，定时测角*/
			f_velocity										=	f_temp / 16.384f;				
		}
	}
	else
	{
		/*使用T法时，如果电机速度测量高于12.5rps时，改用M法*/
		if(((m_motor_rt_para.m_inc_encoder.f_motor_cal_speed > 12.5f) && (m_motor_rt_para.m_inc_encoder.f_motor_cal_speed < -12.5f)) || (f_width == 0) )
		{
			f_temp		=	(float)temp_delta;		
			/*M法测速，定时测角*/
			f_velocity										=	f_temp / 16.384f;
			m_motor_rt_para.m_inc_encoder.u8_M_or_T			=	M_METHORD;
		}
		/*T法测速，定角计时*/
		else
		{			
			f_velocity										=	5126.953125f/((float)f_width);
		}
	}

	//根据方向标志位设置速度正负
	arm_abs_f32(&f_velocity,&(m_motor_rt_para.m_inc_encoder.f_motor_cal_speed),1);
	
	if(m_motor_rt_para.m_inc_encoder.u8_velocity_sign == 0)
		m_motor_rt_para.m_inc_encoder.f_motor_cal_speed		=	-m_motor_rt_para.m_inc_encoder.f_motor_cal_speed;
	else
		m_motor_rt_para.m_inc_encoder.f_motor_cal_speed		=	m_motor_rt_para.m_inc_encoder.f_motor_cal_speed;
	
//	/*除野值*/
//	m_motor_rt_para.m_encoder.f_motor_cal_speed				=	Noise_killer(&nosie_buf,m_motor_rt_para.m_encoder.f_motor_cal_speed);
	
	/*对位置转速进行卡尔曼滤波*/
	
	
	m_motor_ctrl.m_motion_ctrl.u8_speed_read_data_refreshed				=	1;
}



	/*---------------------------------------------------------------------------
	函数名称			：Requir_AbsEncoder(void)
	参数含义			：
	函数功能			：	获取绝对值编码器数据，更新到全局变量中
							开启spi的dma通道，绝对值编码器的接口为SSI，需要等待
							100us左右才能接受到数据
	----------------------------------------------------------------------------*/
void	Requir_AbsEncoder(void)
{
	SPI_DMA_ReadData(4);
}


	/*---------------------------------------------------------------------------
	函数名称			：	Read_AbsEncoder(int32_t * encoder_num)
	参数含义			：
	函数功能			：	将获取的数据写入encoder_num
							解析m_motor_rt_para.m_encoder.u8_abs_raw_data
	----------------------------------------------------------------------------*/
void	Read_AbsEncoder(void)
{
	uint16_t	data_high,data_low ;
	uint16_t	data_1,data_2,data_3;
	int16_t		temp[2];
	
	data_low	=	(uint16_t)(m_motor_rt_para.m_abs_encoder.u8_abs_raw_data[2] << 8) || (m_motor_rt_para.m_abs_encoder.u8_abs_raw_data[3]);
	
	/*查看低9位错误位是否有报警*/
	if(data_low & 0x01FF)																//低9位有警报错误
	{
		m_motor_rt_para.m_abs_encoder.u8_abs_data_refreshed		=	0;
		return;
	}
	else																				//低9位无警报错误
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
		
		m_motor_rt_para.m_abs_encoder.f_abs_pos					=	(float)(m_motor_rt_para.m_abs_encoder.u16_abs_cnt) / 182.0416f;		//当前低速端角度
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
	函数名称			：	Noise_killer(void)
	参数含义			：	buf			存储历史数据的buf
							curr_value	当前的输入值
	函数功能			：	滤除野值
							返回除野值后的数据
							检查数据变化的趋势，用相邻两次数据的差值检查数据合法性
							buf长度大于3
							temp长度大于2
	----------------------------------------------------------------------------*/
uint16_t	Noise_Killer(uint16_t	* buf, uint16_t curr_value,int16_t	*temp)
{	
	if(m_motor_rt_para.m_abs_encoder.u8_abs_history_buf_full > 3)
	{
		temp[1]	=	curr_value - buf[2];						//当前拍增量
		temp[2]	=	buf[2] - buf[1];							//计算上一拍增量
		
		arm_abs_q15(&temp[1],&temp[1],1);						//对增量求绝对值
		arm_abs_q15(&temp[2],&temp[2],1);
		
		if((temp[1] > (temp[2] * 4)) &&(temp[1] > 100))			//增量大于上一拍增量的4倍，并且大于阈值，认为是野值
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

