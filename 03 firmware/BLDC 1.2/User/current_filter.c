/**
  ******************************************************************************
  * @file    Project/user/current_filter.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20180808
  * @brief   电流滤波方法
  ******************************************************************************
  ******************************************************************************
  */

#include	"current_filter.h"
#include	"global_parameters.h"
#include	"stm32f4xx.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
//滑动滤波器 阈值
#define		NOISE_PULSATION_THRESHOLD					500



/* Private variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
uint16_t	Single_Current_Average_X4_Filter(uint8_t channel,uint16_t * new_data);
uint16_t	Single_Current_Average_X8_Filter(uint8_t channel,uint16_t * new_data);


/* Private functions ---------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	函数名称			：Current_Filter_Init(void)
	参数含义			：
	函数功能			：将滤波器内部数据初始化为0
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
	函数名称			：Current_Average_X4_Filter(motor_runtime_para * m_parg)
	参数含义			：
	函数功能			：电流值进行窗口平均滤波，滤波窗口宽度为4，
							采样值超过平均值阈值后舍弃
							进行滤波前，数据存放在m_motor_rt_para.ADC_DMA_buf中
							ADC_DMA_buf 0-2 对应 UVW
							单路数据计算耗时0.8us，pwm周期50us
	----------------------------------------------------------------------------*/
void	Current_Average_X4_Filter(motor_runtime_para * m_parg)
{
	m_parg->u16_u_current	=	Single_Current_Average_X4_Filter(0,&(m_parg->ADC_DMA_buf[0]));
	m_parg->u16_v_current	=	Single_Current_Average_X4_Filter(1,&(m_parg->ADC_DMA_buf[1]));
	m_parg->u16_w_current	=	Single_Current_Average_X4_Filter(2,&(m_parg->ADC_DMA_buf[2]));
	if(m_sys_state.u8_cur_state == Run_state)
	{
		if(m_parg->u16_u_current < m_parg->u16_u_curr_bias)
			m_parg->u16_u_current = m_parg->u16_u_curr_bias;									//电流没有负值，所以采样电流值应大于偏置值
		if(m_parg->u16_v_current < m_parg->u16_v_curr_bias)
			m_parg->u16_v_current = m_parg->u16_v_curr_bias;	
		if(m_parg->u16_w_current < m_parg->u16_w_curr_bias)
			m_parg->u16_w_current = m_parg->u16_w_curr_bias;
	}
		
}

	/*---------------------------------------------------------------------------
	函数名称			：Single_Current_Average_X4_Filter(uint8_t channel,uint16_t * new_data)
	参数含义			：	uint8_t channel		通道UVW
							uint16_t * new_data	新获取数据指针

	函数功能			：  单通道电流值进行窗口平均滤波，滤波窗口宽度为4，
							采样值超过前次值阈值后舍弃
							进行滤波前，数据存放在history_data中
							channel 0-2 对应 UVW
							阈值设定	NOISE_PULSATION_THRESHOLD
							在初始buffer填充时，不考虑阈值问题，如果第一个数据是噪声，将导致滤波器失败
	----------------------------------------------------------------------------*/

uint16_t	Single_Current_Average_X4_Filter(uint8_t channel,uint16_t * new_data)
{
	int16_t		delta_value;											//新数据与上一次采样数据的差值
	int16_t		pre_value;
	uint16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																			//传入的原始采样值不能超过4095
	
	if(m_motor_rt_para.uvw_buffer_used[channel] < 4)																//buffer未填满
	{
		m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]]	=	*new_data;			//将新数据加入buffer
		m_motor_rt_para.uvw_buffer_used[channel]++;
		m_motor_rt_para.filter_index_uvw[channel] 							=	m_motor_rt_para.uvw_buffer_used[channel];			
		m_motor_rt_para.u16_uvw_sum[channel]								+=	*new_data;							//窗口已有数据求和
		return		m_motor_rt_para.u16_uvw_sum[channel]>>2;														//返回平局值
	}
	else																											//buffer已装满
	{
		m_motor_rt_para.filter_index_uvw[channel] ++;																//窗口迁移
		if(m_motor_rt_para.filter_index_uvw[channel] >= 4)
			m_motor_rt_para.filter_index_uvw[channel] =0;
		
		if(m_motor_rt_para.filter_index_uvw[channel] == 3)															//从窗口滑出的值
			abandoned_value			=	m_motor_rt_para.history_data[channel][0];
		else
			abandoned_value			=	m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]+1];
		
		if(m_motor_rt_para.filter_index_uvw[channel] == 0)															//上一次数据
			pre_value				=	(int16_t)m_motor_rt_para.history_data[channel][3];
		else
			pre_value				=	(int16_t)m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel] - 1];
		
		delta_value					=	(int16_t)*new_data - pre_value;												//计算最新数据与上次数据的差值
		
		if((delta_value > NOISE_PULSATION_THRESHOLD) ||(delta_value < -NOISE_PULSATION_THRESHOLD))
			m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]]	=	pre_value;		//新值与前次测量值之差大于阈值，舍弃
		else
			m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]]	=	*new_data;
		
		m_motor_rt_para.u16_uvw_sum[channel]		+=	m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]];
		m_motor_rt_para.u16_uvw_sum[channel]		-=	abandoned_value;
		
		return	m_motor_rt_para.u16_uvw_sum[channel]>>2;
	}
	
}


	/*---------------------------------------------------------------------------
	函数名称			：Current_Average_X8_Filter(motor_runtime_para * m_parg)
	参数含义			：
	函数功能			：电流值进行窗口平均滤波，滤波窗口宽度为8
	----------------------------------------------------------------------------*/
void	Current_Average_X8_Filter(motor_runtime_para * m_parg)
{
	m_parg->u16_u_current	=	Single_Current_Average_X8_Filter(0,&(m_parg->ADC_DMA_buf[0]));
	m_parg->u16_v_current	=	Single_Current_Average_X8_Filter(1,&(m_parg->ADC_DMA_buf[1]));
	m_parg->u16_w_current	=	Single_Current_Average_X8_Filter(2,&(m_parg->ADC_DMA_buf[2]));
}


uint16_t	Single_Current_Average_X8_Filter(uint8_t channel,uint16_t * new_data)
{
	int16_t		delta_value;											//新数据与上一次采样数据的差值
	int16_t		pre_value;
	uint16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																			//传入的原始采样值不能超过4095
	
	if(m_motor_rt_para.uvw_buffer_used[channel] < 8)																//buffer未填满
	{
		m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]]	=	*new_data;							//将新数据加入buffer
		m_motor_rt_para.uvw_buffer_used[channel]++;
		m_motor_rt_para.filter_index_uvw[channel] 							=	m_motor_rt_para.uvw_buffer_used[channel];			
		m_motor_rt_para.u16_uvw_sum[channel]								+=	*new_data;							//窗口已有数据求和
		return		m_motor_rt_para.u16_uvw_sum[channel]>>3;														//返回平局值
	}
	else																							//buffer已装满
	{
		m_motor_rt_para.filter_index_uvw[channel] ++;																//窗口迁移
		if(m_motor_rt_para.filter_index_uvw[channel] >= 8)
			m_motor_rt_para.filter_index_uvw[channel] =0;
		
		if(m_motor_rt_para.filter_index_uvw[channel] == 7)															//从窗口滑出的值
			abandoned_value			=	m_motor_rt_para.history_data[channel][0];
		else
			abandoned_value			=	m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]+1];
		
		if(m_motor_rt_para.filter_index_uvw[channel] == 0)															//上一次数据
			pre_value				=	(int16_t)m_motor_rt_para.history_data[channel][7];
		else
			pre_value				=	(int16_t)m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel] - 1];
		
		delta_value					=	(int16_t)*new_data - pre_value;								//计算最新数据与上次数据的差值
		
		if((delta_value > NOISE_PULSATION_THRESHOLD) ||(delta_value < -NOISE_PULSATION_THRESHOLD))
			m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]]	=	pre_value;						//新值与前次测量值之差大于阈值，舍弃
		else
			m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]]	=	*new_data;
		
		m_motor_rt_para.u16_uvw_sum[channel]		+=	m_motor_rt_para.history_data[channel][m_motor_rt_para.filter_index_uvw[channel]];
		m_motor_rt_para.u16_uvw_sum[channel]		-=	abandoned_value;
		
		return	m_motor_rt_para.u16_uvw_sum[channel]>>3;
	}
}







/*********************************** END OF FILE*****************************************/

