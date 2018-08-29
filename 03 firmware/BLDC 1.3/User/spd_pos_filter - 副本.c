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

//卡尔曼滤波器参数结构体
typedef	struct
{
	float	for_X[2];					//对速度位置的预测量,X1为角度预测值，X2为速度预测值
	float	X_k[2];						//当前拍计算结果
	float	X_k_1[2];					//前一拍计算结果	
	
	float	for_Pk[4];					//k时刻误差的协方差预测值
	float	P_k_1[4];					//k-1时刻计算的误差协方差矩阵	
	float	ctrl_a;						//控制量，加速度
	float	Kk;							//k时刻计算的卡尔曼增益
	float	Q[4];						//系统过程噪声，常量
	float	R[4];						//系统测量噪声
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
	函数名称			：	Noise_killer()
	参数含义			：	noise_buf_struct * noise_buf
							float new_data
	函数功能			：	滤除速度值中的野值
							计算连续几次的速度变化趋势
							出现一次野值将舍弃，用上一拍的数据加上其加速度补充
							效果不好
	----------------------------------------------------------------------------*/
float	Noise_killer(noise_buf_struct * noise_buf, float new_data)
{
	float	f_temp_acc;
	float	f_acc_acc;	
	float	f_temp_acc_acc_abs;

	/*当前拍加速度*/
	f_temp_acc								=	new_data - noise_buf->f_raw_data[1];
	/*计算两次加速度值差的绝对值*/
	f_acc_acc								=	f_temp_acc	-	noise_buf->f_acc;
	arm_abs_f32(&f_acc_acc,&f_temp_acc_acc_abs,1);
	
	if(f_temp_acc_acc_abs > 1.0f)					//当前拍速度增加量大于三倍前一拍速度增加量算作野值，避免acc_abs为0的情况
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
	函数名称			：	KF_Filter()
	参数含义			：	
	函数功能			：	卡尔曼滤波计算，需要先进行滤波器初始化
	----------------------------------------------------------------------------*/
void	KF_Filter(int32_t * input_pos,float * input_spd, int32_t * output_pos, float * output_spd )
{
	
}

	/*---------------------------------------------------------------------------
	函数名称			：Read_IncEncoder(int32_t * encoder_num)
	参数含义			：
	函数功能			：	获取增量编码器数据，更新到全局变量中
	----------------------------------------------------------------------------*/
void	Read_IncEncoder(void)
{
	int32_t		temp_delta;
	uint32_t	f_width		=	0;
	float		f_temp		=	0.0f;
	float		f_velocity	=	0.0f;
	
	m_motor_rt_para.m_encoder.u16_encoder_last_read		=	m_motor_rt_para.m_encoder.u16_encoder_curr_read;						//获取最新编码器读数
	m_motor_rt_para.m_encoder.u16_encoder_curr_read		=	TIM3->CNT;
	
	temp_delta	=	(int32_t)(m_motor_rt_para.m_encoder.u16_encoder_curr_read - m_motor_rt_para.m_encoder.u16_encoder_last_read);
	
	m_motor_rt_para.m_encoder.i32_pulse_cnt				+=	temp_delta;																//获取当前位置信息
	
	/*根据temp_dalta判断转速方向*/
	if(temp_delta < 0)
		m_motor_rt_para.m_encoder.u8_velocity_sign		=	0;
	else
		m_motor_rt_para.m_encoder.u8_velocity_sign		=	1;
	
	/*暂时关闭dma*/
//	TIM_DMACmd(TIM5,TIM_DMA_CC1,DISABLE);
	
	/*对脉宽均值*/
	f_width		=	m_motor_rt_para.m_encoder.u32_pulse_width_buf[1];// + m_motor_rt_para.m_encoder.u32_pulse_width_buf[2] + m_motor_rt_para.m_encoder.u32_pulse_width_buf[3] +m_motor_rt_para.m_encoder.u32_pulse_width_buf[4];
//	f_width		=	f_width>>2;
	
	/*M/T法自动转换 f_shaft_cal_speed*/
	if(m_motor_rt_para.m_encoder.u8_M_or_T == M_METHORD)
	{
		/*使用M法时，如果电机速度测量低于12.5rps时，改用T法*/
		if(((m_motor_rt_para.m_encoder.f_motor_cal_speed < 12.5f) && (m_motor_rt_para.m_encoder.f_motor_cal_speed > -12.5f)) && (f_width != 0))
		{
			/*T法测速，定角计时*/
			// 编码器4096线，四倍频后为16384 Hz，T法测速时检测两个上升沿，
			// 获取脉冲周期 N，则一个脉冲周期时间	t = N / 42000000 秒
			// 码盘转一圈的时间为： t2 = t*16384 / 2 
			// 码盘转速为：1/t2 =	5126.953125 / N
			f_velocity										=	5126.953125f/((float)f_width);
			m_motor_rt_para.m_encoder.u8_M_or_T				=	T_METHORD;
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
		if(((m_motor_rt_para.m_encoder.f_motor_cal_speed > 12.5f) && (m_motor_rt_para.m_encoder.f_motor_cal_speed < -12.5f)) || (f_width == 0) )
		{
			f_temp		=	(float)temp_delta;		
			/*M法测速，定时测角*/
			f_velocity										=	f_temp / 16.384f;
			m_motor_rt_para.m_encoder.u8_M_or_T				=	M_METHORD;
		}
		/*T法测速，定角计时*/
		else
		{			
			f_velocity										=	5126.953125f/((float)f_width);
		}
	}
	/*开启DMA通道*/
//	TIM_DMACmd(TIM5,TIM_DMA_CC1,ENABLE);
	
	//根据方向标志位设置速度正负
	arm_abs_f32(&f_velocity,&(m_motor_rt_para.m_encoder.f_motor_cal_speed),1);
	
	if(m_motor_rt_para.m_encoder.u8_velocity_sign == 0)
		m_motor_rt_para.m_encoder.f_motor_cal_speed			=	-m_motor_rt_para.m_encoder.f_motor_cal_speed;
	else
		m_motor_rt_para.m_encoder.f_motor_cal_speed			=	m_motor_rt_para.m_encoder.f_motor_cal_speed;
	
	/*除野值*/
//	m_motor_rt_para.m_encoder.f_motor_cal_speed				=	Noise_killer(&nosie_buf,m_motor_rt_para.m_encoder.f_motor_cal_speed);
	
	m_motor_ctrl.u8_speed_read_data_refreshed				=	1;
}








/*****************************END OF FILE************************************/

