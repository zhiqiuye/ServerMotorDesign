/**
  ******************************************************************************
  * @file    Project/user/current_filter.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20180808
  * @brief   电流滤波方法
  *
  *				在bldc电流值采样过程中，使用三个功率电阻在下桥臂测量，每个电阻测到
  *			其通路下桥臂导通时的电流值，因此，需要知道当前导通相位，然后取到响应的
  *			导通电流作为相电流。
  *				开始做电流滤波器时，对三路电流分别进行窗口平均滤波，这样意味着将三
  *			路换向时的电流变化引入到最终获得的相电流中。
  *				正确做法应该是每次选择正确的一路相电流原始值加入滤波器
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
//滑动滤波器 阈值
#define		NOISE_PULSATION_THRESHOLD					50



/* Private variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
uint16_t	Single_Current_Average_X4_Filter(uint16_t * new_data);
int16_t		Single_Current_Average_X8_Filter(int16_t * new_data);


/* Private functions ---------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	函数名称			：Current_Filter_Init(void)
	参数含义			：
	函数功能			：将滤波器内部数据初始化为0
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
	函数名称			：Current_Average_X4_Filter(motor_runtime_para * m_parg)
	参数含义			：
	函数功能			：电流值进行窗口平均滤波，滤波窗口宽度为4，
							采样值超过平均值阈值后舍弃
							进行滤波前，数据存放在m_motor_rt_para.ADC_DMA_buf中
							ADC_DMA_buf 0-2 对应 UVW
							单路数据计算耗时0.8us，pwm周期50us
	----------------------------------------------------------------------------*/
void	Current_Average_X4_Filter(current_sensor_state * m_parg)
{
	int16_t		u_i,v_i,w_i;
	uint16_t	real_i;
	/*计算u相电流绝对值，整型变量*/
	u_i							=	m_parg->ADC_DMA_buf[0] - m_parg->i16_uvw_curr_bias;
	u_i							=	((u_i) > 0 ? (u_i) : -(u_i) );
	/*计算u相电流绝对值，整型变量*/
	v_i							=	m_parg->ADC_DMA_buf[1] - m_parg->i16_uvw_curr_bias;
	v_i							=	((v_i) > 0 ? (v_i) : -(v_i) );
	/*计算u相电流绝对值，整型变量*/
	w_i							=	m_parg->ADC_DMA_buf[2] - m_parg->i16_uvw_curr_bias;
	w_i							=	((w_i) > 0 ? (w_i) : -(w_i) );	
	/*计算三相平均*/
	real_i						=	(u_i + v_i + w_i)>>1;
	/*压入滤波器*/
	m_parg->i16_uvw_current		=	Single_Current_Average_X4_Filter(&real_i);
	
	if(m_motor_ctrl.m_sys_state.u8_cur_state == Run_state)	
	{
		if(m_parg->i16_uvw_current > 2000)
			m_parg->i16_uvw_current = 2000;												//设置电流值上限
	}
}

	/*---------------------------------------------------------------------------
	函数名称			：Single_Current_Average_X4_Filter(uint16_t * new_data)
	参数含义			：	
							uint16_t * new_data	新获取数据指针

	函数功能			：  选择相应的通道数据，添加入databuf中进行平均
	----------------------------------------------------------------------------*/

uint16_t	Single_Current_Average_X4_Filter(uint16_t * new_data)
{
//	int16_t		delta_value;																											//新数据与上一次采样数据的差值
//	int16_t		pre_value;
	uint16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																												//传入的原始采样值不能超过4095
	
	if(m_motor_rt_para.m_current_filter.uvw_buffer_used < 4)																			//buffer未填满
	{
		m_motor_rt_para.m_current_filter.history_data[m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*new_data;				//将新数据加入buffer
		m_motor_rt_para.m_current_filter.uvw_buffer_used++;
		m_motor_rt_para.m_current_filter.filter_index_uvw 								=	m_motor_rt_para.m_current_filter.uvw_buffer_used;			
		m_motor_rt_para.m_current_filter.i16_uvw_sum									+=	*new_data;									//窗口已有数据求和
		return		m_motor_rt_para.m_current_filter.i16_uvw_sum>>2;																	//返回平局值
	}
	else																																//buffer已装满
	{
		m_motor_rt_para.m_current_filter.filter_index_uvw++;																			//窗口迁移
		if(m_motor_rt_para.m_current_filter.filter_index_uvw >= 4)
			m_motor_rt_para.m_current_filter.filter_index_uvw =0;
		
		if(m_motor_rt_para.m_current_filter.filter_index_uvw == 3)																		//从窗口滑出的值
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
	函数名称			：Current_Average_X8_Filter(motor_runtime_para * m_parg)
	参数含义			：
	函数功能			：电流值进行窗口平均滤波，滤波窗口宽度为8
	----------------------------------------------------------------------------*/
void	Current_Average_X8_Filter(current_sensor_state * m_parg)
{
//	int16_t		u_i,v_i,w_i;
	int16_t		real_i;
//	/*计算u相电流绝对值，整型变量*/
//	u_i							=	m_parg->ADC_DMA_buf[0] - m_parg->i16_uvw_curr_bias;	
//	u_i							=	((u_i) > 0 ? (u_i) : -(u_i) );//((m_parg->ADC_DMA_buf[0]>m_parg->i16_uvw_curr_bias)?(u_i) : -(u_i));//
//	/*计算v相电流绝对值，整型变量*/
//	v_i							=	m_parg->ADC_DMA_buf[1] - m_parg->i16_uvw_curr_bias;
//	v_i							=	((v_i) > 0 ? (v_i) : -(v_i) );
//	/*计算w相电流绝对值，整型变量*/
//	w_i							=	m_parg->ADC_DMA_buf[2] - m_parg->i16_uvw_curr_bias;
//	w_i							=	((w_i) > 0 ? (w_i) : -(w_i) );	
//	/*计算三相平均*/
//	real_i						=	(u_i + v_i + w_i)>>1;
//	/*压入滤波器*/
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
			m_parg->i16_uvw_current = 2000;																		//设置电流值上限
	}
}


int16_t	Single_Current_Average_X8_Filter(int16_t * new_data)
{
	int16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																						//传入的原始采样值不能超过4095
	
	if(m_motor_rt_para.m_current_filter.uvw_buffer_used < 8)													//buffer未填满
	{
		m_motor_rt_para.m_current_filter.history_data[m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*new_data;								//将新数据加入buffer
		m_motor_rt_para.m_current_filter.uvw_buffer_used++;
		m_motor_rt_para.m_current_filter.filter_index_uvw 								=	m_motor_rt_para.m_current_filter.uvw_buffer_used;			
		m_motor_rt_para.m_current_filter.i16_uvw_sum									+=	*new_data;			//窗口已有数据求和
		return		m_motor_rt_para.m_current_filter.i16_uvw_sum>>3;											//返回平局值
	}
	else																										//buffer已装满
	{
		m_motor_rt_para.m_current_filter.filter_index_uvw++;													//窗口迁移
		if(m_motor_rt_para.m_current_filter.filter_index_uvw >= 8)
			m_motor_rt_para.m_current_filter.filter_index_uvw =0;
		
		if(m_motor_rt_para.m_current_filter.filter_index_uvw == 7)												//从窗口滑出的值
			abandoned_value			=	m_motor_rt_para.m_current_filter.history_data[0];
		else
			abandoned_value			=	m_motor_rt_para.m_current_filter.history_data[m_motor_rt_para.m_current_filter.filter_index_uvw + 1];
		
		/*除野值*/
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

