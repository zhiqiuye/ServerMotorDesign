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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
//滑动滤波器 阈值
#define		NOISE_PULSATION_THRESHOLD					300



/* Private variables ---------------------------------------------------------*/
//电流相序表
extern const	uint8_t		current_senser_table[2][7] ;										



/* Private function prototypes -----------------------------------------------*/
uint16_t	Single_Current_Average_X4_Filter(uint16_t * new_data);
uint16_t	Single_Current_Average_X8_Filter(uint16_t * new_data);


/* Private functions ---------------------------------------------------------*/
	/*---------------------------------------------------------------------------
	函数名称			：Current_Filter_Init(void)
	参数含义			：
	函数功能			：将滤波器内部数据初始化为0
	----------------------------------------------------------------------------*/
void	Current_Filter_Init(void)
{
	m_motor_rt_para.u16_uvw_sum				=	0;
	m_motor_rt_para.filter_index_uvw		=	0;
	m_motor_rt_para.uvw_buffer_used			=	0;
	m_motor_rt_para.u16_uvw_curr_bias		=	0;
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
	int16_t		u_i,v_i,w_i;
	uint16_t	real_i;
	/*计算u相电流绝对值，整型变量*/
	u_i							=	m_parg->ADC_DMA_buf[0] - m_parg->u16_uvw_curr_bias;
	u_i							=	((u_i) > 0 ? (u_i) : -(u_i) );
	/*计算u相电流绝对值，整型变量*/
	v_i							=	m_parg->ADC_DMA_buf[1] - m_parg->u16_uvw_curr_bias;
	v_i							=	((v_i) > 0 ? (v_i) : -(v_i) );
	/*计算u相电流绝对值，整型变量*/
	w_i							=	m_parg->ADC_DMA_buf[2] - m_parg->u16_uvw_curr_bias;
	w_i							=	((w_i) > 0 ? (w_i) : -(w_i) );	
	/*计算三相平均*/
	real_i						=	(u_i + v_i + w_i)>>1;
	/*压入滤波器*/
	m_parg->u16_uvw_current		=	Single_Current_Average_X4_Filter(&real_i);
	
	if(m_sys_state.u8_cur_state == Run_state)	
	{
		if(m_parg->u16_uvw_current > 2000)
			m_parg->u16_uvw_current = 2000;												//设置电流值上限
	}
	
	m_motor_ctrl.u8_current_read_data_refreshed		=	1;								//读取电流反馈数据更新
}

	/*---------------------------------------------------------------------------
	函数名称			：Single_Current_Average_X4_Filter(uint16_t * new_data)
	参数含义			：	
							uint16_t * new_data	新获取数据指针

	函数功能			：  选择相应的通道数据，添加入databuf中进行平均
	----------------------------------------------------------------------------*/

uint16_t	Single_Current_Average_X4_Filter(uint16_t * new_data)
{
//	int16_t		delta_value;															//新数据与上一次采样数据的差值
//	int16_t		pre_value;
	uint16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																//传入的原始采样值不能超过4095
	
	if(m_motor_rt_para.uvw_buffer_used < 4)																			//buffer未填满
	{
		m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw]	=	*new_data;								//将新数据加入buffer
		m_motor_rt_para.uvw_buffer_used++;
		m_motor_rt_para.filter_index_uvw 								=	m_motor_rt_para.uvw_buffer_used;			
		m_motor_rt_para.u16_uvw_sum										+=	*new_data;								//窗口已有数据求和
		return		m_motor_rt_para.u16_uvw_sum>>2;																	//返回平局值
	}
	else																											//buffer已装满
	{
		m_motor_rt_para.filter_index_uvw++;																			//窗口迁移
		if(m_motor_rt_para.filter_index_uvw >= 4)
			m_motor_rt_para.filter_index_uvw =0;
		
		if(m_motor_rt_para.filter_index_uvw == 3)																	//从窗口滑出的值
			abandoned_value			=	m_motor_rt_para.history_data[0];
		else
			abandoned_value			=	m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw + 1];
		
//		if(m_motor_rt_para.filter_index_uvw == 0)															//上一次数据
//			pre_value				=	(int16_t)m_motor_rt_para.history_data[3];
//		else
//			pre_value				=	(int16_t)m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw - 1];
//		
//		delta_value					=	(int16_t)*new_data - pre_value;												//计算最新数据与上次数据的差值
//		
//		if((delta_value > NOISE_PULSATION_THRESHOLD) ||(delta_value < -NOISE_PULSATION_THRESHOLD))
//			m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw]	=	pre_value;		//新值与前次测量值之差大于阈值，舍弃
//		else
			m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw]	=	*new_data;
		
		m_motor_rt_para.u16_uvw_sum		+=	m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw];
		m_motor_rt_para.u16_uvw_sum		-=	abandoned_value;
		
		return	m_motor_rt_para.u16_uvw_sum>>2;
	}
	
}


	/*---------------------------------------------------------------------------
	函数名称			：Current_Average_X8_Filter(motor_runtime_para * m_parg)
	参数含义			：
	函数功能			：电流值进行窗口平均滤波，滤波窗口宽度为8
	----------------------------------------------------------------------------*/
void	Current_Average_X8_Filter(motor_runtime_para * m_parg)
{
	int16_t		u_i,v_i,w_i;
	uint16_t	real_i;
	/*计算u相电流绝对值，整型变量*/
	u_i							=	m_parg->ADC_DMA_buf[0] - m_parg->u16_uvw_curr_bias;	
	u_i							=	((u_i) > 0 ? (u_i) : -(u_i) );
	/*计算v相电流绝对值，整型变量*/
	v_i							=	m_parg->ADC_DMA_buf[1] - m_parg->u16_uvw_curr_bias;
	v_i							=	((v_i) > 0 ? (v_i) : -(v_i) );
	/*计算w相电流绝对值，整型变量*/
	w_i							=	m_parg->ADC_DMA_buf[2] - m_parg->u16_uvw_curr_bias;
	w_i							=	((w_i) > 0 ? (w_i) : -(w_i) );	
	/*计算三相平均*/
	real_i						=	(u_i + v_i + w_i)>>1;
	/*压入滤波器*/
	m_parg->u16_uvw_current		=	Single_Current_Average_X8_Filter(&real_i);
	
	if(m_sys_state.u8_cur_state == Run_state)	
	{
		if(m_parg->u16_uvw_current > 2000)
			m_parg->u16_uvw_current = 2000;												//设置电流值上限
	}
	
	m_motor_ctrl.u8_current_read_data_refreshed		=	1;								//读取电流反馈数据更新
}


uint16_t	Single_Current_Average_X8_Filter(uint16_t * new_data)
{
	uint16_t	abandoned_value;
	
	if(*new_data > 4095)
		*new_data = 4095;																//传入的原始采样值不能超过4095
	
	if(m_motor_rt_para.uvw_buffer_used < 8)																			//buffer未填满
	{
		m_motor_rt_para.history_data[m_motor_rt_para.filter_index_uvw]	=	*new_data;								//将新数据加入buffer
		m_motor_rt_para.uvw_buffer_used++;
		m_motor_rt_para.filter_index_uvw 								=	m_motor_rt_para.uvw_buffer_used;			
		m_motor_rt_para.u16_uvw_sum										+=	*new_data;								//窗口已有数据求和
		return		m_motor_rt_para.u16_uvw_sum>>2;																	//返回平局值
	}
	else																											//buffer已装满
	{
		m_motor_rt_para.filter_index_uvw++;																			//窗口迁移
		if(m_motor_rt_para.filter_index_uvw >= 8)
			m_motor_rt_para.filter_index_uvw =0;
		
		if(m_motor_rt_para.filter_index_uvw == 7)																	//从窗口滑出的值
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

