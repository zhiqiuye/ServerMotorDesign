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


#ifndef		__GLOBAL_PARAMETERS_H
#define		__GLOBAL_PARAMETERS_H

#include	"stm32f4xx.h"

/*�궨��------------------------------------------------------------------------------*/
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





/*����������---------------------------------------------------------------------------*/
typedef	struct
{
	uint32_t	u32_line_per_loop;						//ÿȦ����
	uint8_t		u8_multiplication;						//��Ƶ��
	uint32_t	u32_pulse_per_loop;						//ÿȦ������
}encoder_para;

/*����̶�����--------------------------------------------------------------------------*/
typedef	struct
{
	encoder_para	m_encoder;							//����������
	uint32_t		u32_maxspeed;						//����ٶȣ�rpm
	float			f_no_load_speed;					//����ת�٣�rpm
	float			f_no_load_current;					//���ص�����A
	float			f_stall_torque;						//��תת�أ�mNm
	float			f_speed_constant;					//ת�ٳ�����rpm/V
	float			f_torque_constant;					//ת�س�����mNm/A
	float			f_current_constant;					//����������A/mNm
	float			f_terminal_inductance;				//���У�uH
	float			f_mechanical_time_constant;			//��еʱ�䳣����ms
	uint8_t			u8_number_of_pole_pairs;			//�ż�����
	
}const_motor_para;

/*�����ת����--------------------------------------------------------------------------*/
typedef	struct
{
	uint16_t		ADC_DMA_buf[32];					//dma���ݴ��buf��ԭʼ���ݣ�ÿ�����ݲɼ�4����ƽ��
	uint16_t		u16_power_voltage;					//��Դ��ѹ
	uint16_t		u16_u_current;						//U�����
	uint16_t		u16_v_current;						//V�����
	uint16_t		u16_w_current;						//w�����
	
	//-------------------------------------test para
	float			f_adc1;
	float			f_adc2;
	float			f_adc3;
	float			f_adc4;
}motor_runtime_para;


/*���Ʋ���------------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		u8_dir;									//ת������
	float		f_set_current;							//���õ���ֵ��������		����ȷ��dir��int_set_current
	uint32_t	u32_set_current;						//���õ���ֵ���޷�������
	float		f_set_speed;							//�����ٶ�ֵ��������
	uint32_t	u32_set_speed;							//�����ٶ�ֵ���޷�������	
	float		f_set_position;							//����λ��ֵ��������
	uint32_t	u32_set_position;						//����λ��ֵ���޷�������
}motor_control_para;


/*����������------------------------------------------------------------------------------*/
typedef	struct
{
	float		err_p;
}current_pid_para;

/*�ٶȻ�����-------------------------------------------------------------------------------*/
typedef	struct
{
	float		err_p;
}speed_pid_para;



/*λ�û�����--------------------------------------------------------------------------------*/
typedef	struct
{
	float		err_p;
}position_pid_para;



/*������Ϣ--------------------------------------------------------------------------------*/
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











