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


#ifndef		__GLOBAL_PARAMETERS_H
#define		__GLOBAL_PARAMETERS_H

#include	"stm32f4xx.h"

/*宏定义------------------------------------------------------------------------------*/
#define		MAX_DUTY_CYCLE		4200
#define		MIN_DUTY_CYCLE		0
		
enum BRAKE_ERR
{
	control_floating = 1,
	open_error,
	close_error,
};

enum VOLTAGE_ERR
{
	over_voltage = 1,
	under_voltage,
};





/*编码器参数---------------------------------------------------------------------------*/
typedef	struct
{
	uint32_t	u32_line_per_loop;						//每圈线数
	uint8_t		u8_multiplication;						//倍频数
	uint32_t	u32_pulse_per_loop;						//每圈脉冲数
}encoder_para;

/*电机固定参数--------------------------------------------------------------------------*/
typedef	struct
{
	encoder_para	m_encoder;							//编码器数据
	uint32_t		u32_maxspeed;						//最大速度，rpm
	float			f_no_load_speed;					//空载转速，rpm
	float			f_no_load_current;					//空载电流，A
	float			f_stall_torque;						//堵转转矩，mNm
	float			f_speed_constant;					//转速常数，rpm/V
	float			f_torque_constant;					//转矩常数，mNm/A
	float			f_current_constant;					//电流常数，A/mNm
	float			f_terminal_inductance;				//相电感，uH
	float			f_mechanical_time_constant;			//机械时间常数，ms
	uint8_t			u8_number_of_pole_pairs;			//磁极对数
	
}const_motor_para;

/*电机运转参数--------------------------------------------------------------------------*/
typedef	struct
{
	uint16_t		ADC_DMA_buf[32];					//dma数据存放buf，原始数据，每个数据采集4次做平均
	uint16_t		u16_power_voltage;					//电源电压
	uint16_t		u16_u_current;						//U相电流
	uint16_t		u16_v_current;						//V相电流
	uint16_t		u16_w_current;						//w相电流
	
	//-------------------------------------test para
	float			f_adc1;
	float			f_adc2;
	float			f_adc3;
	float			f_adc4;
}motor_runtime_para;


/*控制参数------------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		u8_dir;									//转动方向
	float		f_set_current;							//设置电流值，浮点型		正负确定dir和int_set_current
	uint32_t	u32_set_current;						//设置电流值，无符号整形
	float		f_set_speed;							//设置速度值，浮点型
	uint32_t	u32_set_speed;							//设置速度值，无符号整形	
	float		f_set_position;							//设置位置值，浮点型
	uint32_t	u32_set_position;						//设置位置值，无符号整形
}motor_control_para;


/*电流环参数------------------------------------------------------------------------------*/
typedef	struct
{
	float		err_p;
}current_pid_para;

/*速度环参数-------------------------------------------------------------------------------*/
typedef	struct
{
	float		err_p;
}speed_pid_para;



/*位置环参数--------------------------------------------------------------------------------*/
typedef	struct
{
	float		err_p;
}position_pid_para;



/*错误信息--------------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		brake_err;
	uint8_t		power_error;
	
}error_log;







/*GLOBAL PARAMETERS**************************************************************************/
extern	const_motor_para			m_motor;

extern	motor_runtime_para			m_motor_rt_para;

extern	motor_control_para			m_motor_ctrl;

extern	current_pid_para			m_current_pid;

extern	speed_pid_para				m_speed_pid;

extern	position_pid_para			m_position_pid;

extern	error_log					m_error;

/*FUNCTIONS**********************************************************************************/
uint8_t	ParametersInit(void);

uint8_t	ParametersSave(void);

uint8_t	ParametersCheck(void);



#endif

/**********************end of file************************/











