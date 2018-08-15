/**
  ******************************************************************************
  * @file    Project/ucosii stm32f4 mdk475/global_parameters.h 
  * @author  Kuangjing
  * @version ucosii
  * @date    20180619
  * @brief   ����ȫ�ֱ�����������������Ʋ�����.
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
	��������			��ParametersInit(void)
	��������			��null
	��������			����ʼ��������
	----------------------------------------------------------------------------*/
uint8_t	ParametersInit(void)
{
	m_motor_ctrl.u8_dir	=	0;										//���õ��ת��
	
	return	1;
}




	/*---------------------------------------------------------------------------
	��������			��ParametersSave(void)
	��������			��null
	��������			����������������flash��
	----------------------------------------------------------------------------*/
uint8_t	ParametersSave(void)
{

	return	1;
}


	/*---------------------------------------------------------------------------
	��������			��ParametersCheck(void)
	��������			��null
	��������			������������Ƿ�Ϸ�
	----------------------------------------------------------------------------*/
uint8_t	ParametersCheck(void)
{

	return	1;
}





/**********************end of file************************/

