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
#include	"global_parameters.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

	/*---------------------------------------------------------------------------
	函数名称			：Read_IncEncoder(int32_t * encoder_num)
	参数含义			：
	函数功能			：	获取增量编码器数据，更新到全局变量中
	----------------------------------------------------------------------------*/
void	Read_IncEncoder(void)
{
	int32_t		temp_delta;
	
	m_motor_rt_para.i32_encoder_last_read	=	m_motor_rt_para.i32_encoder_curr_read;		//获取最新编码器读数
	m_motor_rt_para.i32_encoder_curr_read	=	TIM3->CNT;
	
	temp_delta	=	m_motor_rt_para.i32_encoder_curr_read - m_motor_rt_para.i32_encoder_last_read;
	
	if(temp_delta < -40000)						//计数器向上溢出，重新计数
	{
		m_motor_rt_para.f_motor_cal_speed	=	((float)(temp_delta + 65535))/40.0f;		///40000*1000 获得rps单位的速度
	}
	else if(temp_delta > 40000)					//计数器反向计数，减过0
	{
		m_motor_rt_para.f_motor_cal_speed	=	((float)(65535 - temp_delta))/40.0f;		///40000*1000 获得rps单位的速度
	}
	else									//计数器在0-65535之间正常计数
	{
		m_motor_rt_para.f_motor_cal_speed	=	((float)temp_delta)/40.0f;					///40000*1000 获得rps单位的速度
	}
}








/*****************************END OF FILE************************************/

