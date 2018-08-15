/**
  ******************************************************************************
  * @file    Project/ucosii stm32f4 mdk475/global_parameters.h 
  * @author  Kuangjing
  * @version ucosii
  * @date    20180619
  * @brief   定义全局变量，电机参数，控制参数等.
  ******************************************************************************
  ******************************************************************************
  */

#include	"global_parameters.h"
#include	"stm32f4xx.h"



const_motor_para		m_motor;

motor_runtime_para		m_motor_rt_para;

motor_control_para		m_motor_ctrl;

current_pid_para		m_current_pid;

speed_pid_para			m_speed_pid;

position_pid_para		m_position_pid;

error_log				m_error;


	/*---------------------------------------------------------------------------
	函数名称			：ParametersInit(void)
	参数含义			：null
	函数功能			：初始化各参数
	----------------------------------------------------------------------------*/
uint8_t	ParametersInit(void)
{
	m_motor_ctrl.u8_dir	=	0;										//设置电机转向
	
	return	1;
}




	/*---------------------------------------------------------------------------
	函数名称			：ParametersSave(void)
	参数含义			：null
	函数功能			：将电机参数存放至flash中
	----------------------------------------------------------------------------*/
uint8_t	ParametersSave(void)
{

	return	1;
}


	/*---------------------------------------------------------------------------
	函数名称			：ParametersCheck(void)
	参数含义			：null
	函数功能			：检查电机参数是否合法
	----------------------------------------------------------------------------*/
uint8_t	ParametersCheck(void)
{

	return	1;
}





/**********************end of file************************/

