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


/*	六步换向时，各电流传感器采集的电流值与真实电流值关系
-------------------------------------------------------------------------------
	正转时：
			Hall State			Phase A		Phase B		Phase C
				1					0			-I			I
				2					-I			I			0
				3					-I			0			I
				4					I			0			-I
				5					I			-I			0
				6					0			I			-I

	反转时：
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
//滑动滤波器 阈值
#define		NOISE_PULSATION_THRESHOLD					50



/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void		Single_Current_Average_X8_Filter_6Step(int16_t * new_data);
void		Single_Current_Average_X8_Filter_SVPWM(int16_t * u_data,int16_t * v_data,int16_t * w_data);



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
	函数名称			：	Current_Average_X8_Filter(motor_runtime_para * m_parg)
	参数含义			：
	函数功能			：	电流值进行窗口平均滤波，滤波窗口宽度为8
						采样值超过平均值阈值后舍弃
						进行滤波前，数据存放在m_motor_rt_para.ADC_DMA_buf中
						ADC_DMA_buf 0-2 对应 UVW
						单路数据计算耗时0.8us，pwm周期50us
	----------------------------------------------------------------------------*/
void	Current_Average_X8_Filter(current_sensor_state * m_parg)
{
	int16_t		real_i;
	int16_t		real_u,real_v,real_w;
	/*梯形脉冲时电流的滤波*/
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
				m_parg->i16_uvw_current = 2000;																		//设置电流值上限
		}	
	}
	/*SVPWM时的电流滤波*/
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
		*new_data = 4095;																						//传入的原始采样值不能超过4095
	
	if(m_motor_rt_para.m_current_filter.uvw_buffer_used < 8)													//buffer未填满
	{
		m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*new_data;								//将新数据加入buffer
		m_motor_rt_para.m_current_filter.uvw_buffer_used++;
		m_motor_rt_para.m_current_filter.filter_index_uvw 								=	m_motor_rt_para.m_current_filter.uvw_buffer_used;			
		m_motor_rt_para.m_current_filter.i16_uvw_sum									+=	*new_data;			//窗口已有数据求和
		m_motor_rt_para.m_current_sensor.i16_uvw_current	=	m_motor_rt_para.m_current_filter.i16_uvw_sum / m_motor_rt_para.m_current_filter.uvw_buffer_used;	//返回平局值
	}
	else																										//buffer已装满
	{
		m_motor_rt_para.m_current_filter.filter_index_uvw++;													//窗口迁移
		if(m_motor_rt_para.m_current_filter.filter_index_uvw >= 8)
			m_motor_rt_para.m_current_filter.filter_index_uvw =0;
		
		if(m_motor_rt_para.m_current_filter.filter_index_uvw == 7)												//从窗口滑出的值
			abandoned_value			=	m_motor_rt_para.m_current_filter.history_data[0][0];
		else
			abandoned_value			=	m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw + 1];
		
		/*除野值*/
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
	
	if(*u_data > 4095)	*u_data = 4095;																			//传入的原始采样值不能超过4095
	if(*v_data > 4095)	*v_data = 4095;																			//传入的原始采样值不能超过4095
	if(*w_data > 4095)	*w_data = 4095;																			//传入的原始采样值不能超过4095
	
	if(m_motor_rt_para.m_current_filter.uvw_buffer_used < 8)													//buffer未填满
	{
		m_motor_rt_para.m_current_filter.history_data[0][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*u_data;								//将新数据加入buffer
		m_motor_rt_para.m_current_filter.history_data[1][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*v_data;	
		m_motor_rt_para.m_current_filter.history_data[2][m_motor_rt_para.m_current_filter.filter_index_uvw]	=	*w_data;	
		m_motor_rt_para.m_current_filter.uvw_buffer_used++;
		m_motor_rt_para.m_current_filter.filter_index_uvw 								=	m_motor_rt_para.m_current_filter.uvw_buffer_used;			
		m_motor_rt_para.m_current_filter.i16_u_sum										+=	*u_data;			//窗口已有数据求和
		m_motor_rt_para.m_current_filter.i16_v_sum										+=	*v_data;			//窗口已有数据求和
		m_motor_rt_para.m_current_filter.i16_w_sum										+=	*w_data;			//窗口已有数据求和
		
		m_motor_rt_para.m_current_sensor.i16_u_current	=	m_motor_rt_para.m_current_filter.i16_u_sum / m_motor_rt_para.m_current_filter.uvw_buffer_used;
		m_motor_rt_para.m_current_sensor.i16_v_current	=	m_motor_rt_para.m_current_filter.i16_v_sum / m_motor_rt_para.m_current_filter.uvw_buffer_used;
		m_motor_rt_para.m_current_sensor.i16_w_current	=	m_motor_rt_para.m_current_filter.i16_w_sum / m_motor_rt_para.m_current_filter.uvw_buffer_used;
	}
	else																										//buffer已装满
	{
		m_motor_rt_para.m_current_filter.filter_index_uvw++;													//窗口迁移
		if(m_motor_rt_para.m_current_filter.filter_index_uvw >= 8)
			m_motor_rt_para.m_current_filter.filter_index_uvw =0;
		
		if(m_motor_rt_para.m_current_filter.filter_index_uvw == 7)												//从窗口滑出的值
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
		/*除野值*/
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
		
		/*计算各相电流值*/
		m_motor_rt_para.m_current_sensor.i16_u_current	=	m_motor_rt_para.m_current_filter.i16_u_sum>>3;
		m_motor_rt_para.m_current_sensor.i16_v_current	=	m_motor_rt_para.m_current_filter.i16_v_sum>>3;
		m_motor_rt_para.m_current_sensor.i16_w_current	=	m_motor_rt_para.m_current_filter.i16_w_sum>>3;
	}
}




/*********************************** END OF FILE*****************************************/

