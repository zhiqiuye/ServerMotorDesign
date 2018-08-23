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



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

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
	函数名称			：Read_IncEncoder(int32_t * encoder_num)
	参数含义			：
	函数功能			：	获取增量编码器数据，更新到全局变量中
	----------------------------------------------------------------------------*/
void	Read_IncEncoder(void)
{
	int32_t		temp_delta;
	float		f_temp	=	0.0f;
	uint32_t	f_width	=	0;
	
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
	TIM_DMACmd(TIM5,TIM_DMA_CC1,DISABLE);
	
	/*对脉宽均值*/
	
	
	/*M/T法自动转换 f_shaft_cal_speed*/
	if(m_motor_rt_para.m_encoder.u8_M_or_T == M_METHORD)
	{
		/*使用M法时，如果电机速度测量低于12.5rps时，改用T法*/
		if(((m_motor_rt_para.m_encoder.f_motor_cal_speed < 12.5f) && (m_motor_rt_para.m_encoder.f_motor_cal_speed > -12.5f)) && (m_motor_rt_para.m_encoder.u32_pulse_width_buf[1] != 0))
		{
			/*T法测速，定角计时*/
			// 编码器4096线，四倍频后为16384 Hz，T法测速时检测两个上升沿，
			// 获取脉冲周期 N，则一个脉冲周期时间	t = N / 42000000 秒
			// 码盘转一圈的时间为： t2 = t*16384 / 2 
			// 码盘转速为：1/t2 =	5126.953125 / N
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
//													&m_motor_rt_para.m_encoder.i32_spd_his_sum);														//速度均值滤波	
			f_temp		=	(float)temp_delta;		
			/*M法测速，定时测角*/
			m_motor_rt_para.m_encoder.f_motor_cal_speed			=	f_temp / 16.384f;				
		}
	}
	else
	{
		/*使用T法时，如果电机速度测量高于12.5rps时，改用M法*/
		if(((m_motor_rt_para.m_encoder.f_motor_cal_speed > 12.5f) && (m_motor_rt_para.m_encoder.f_motor_cal_speed < -12.5f)) || (m_motor_rt_para.m_encoder.u32_pulse_width_buf[1] == 0) )
		{
			f_temp		=	(float)temp_delta;		
			/*M法测速，定时测角*/
			m_motor_rt_para.m_encoder.f_motor_cal_speed			=	f_temp / 16.384f;
			m_motor_rt_para.m_encoder.u8_M_or_T					=	M_METHORD;
		}
		/*T法测速，定角计时*/
		else
		{			
			if(m_motor_rt_para.m_encoder.u8_velocity_sign == 1)
				m_motor_rt_para.m_encoder.f_motor_cal_speed		=	5126.953125f/((float)m_motor_rt_para.m_encoder.u32_pulse_width_buf[1]);
			else
				m_motor_rt_para.m_encoder.f_motor_cal_speed		=	-5126.953125f/((float)m_motor_rt_para.m_encoder.u32_pulse_width_buf[1]);
		}
	}
	/*开启DMA通道*/
	TIM_DMACmd(TIM5,TIM_DMA_CC1,ENABLE);
	
	m_motor_ctrl.u8_speed_read_data_refreshed					=	1;
}








/*****************************END OF FILE************************************/

