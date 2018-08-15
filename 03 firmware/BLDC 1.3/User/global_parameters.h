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



/* Private macro -------------------------------------------------------------*/
//encoder
#define		LINES_PER_LOOP								4096
#define		MULTIPLICATION								4

//bldc
#define		V_bus										24				//������ѹ
#define		N_Rated										2450			//������µ��ٶȣ�rpm
#define		Ip											38.0f			//��ֵ������A
#define		Ic											10.1f			//����������A
#define		Tc											0.706f			//����ʧ��ת�أ�Nm
#define		Tp											2.56f			//��ֵʧ��ת�أ�Nm
#define		Kt_hot										0.074f			//155��ת�������ȣ�Nm/A
#define		Kt_cold										0.081f			//25��ת�������ȣ�Nm/A
#define		Kb_hot										7.79f			//���綯�Ƴ�����V/krpm
#define		Kb_cold										8.57f			//���綯�Ƴ�����V/krpm
#define		P											12				//����
#define		Lm											0.18			//��У�mH

//״̬����
#define		Idle_state									0				//�ϵ���ڣ�����δ׼����ϣ��ڴ�״̬
#define		Run_state									1				//��������״̬
#define		Prepare_state								2				//�ϵ�󣬸�ģ���ʼ����ϣ������״̬
#define		Halt_state									3				//��������״̬����ֹͣ
//        |-----------�ϵ�                                                           
//        |           |------�����ʼ��δ���                                                
//        |           |                |---------�����ʼ�����                                
//        |           |                |            |---------������ת״̬                    
//        |           |                |            |             |---------��ͣ�׶�       
//		START -> Idle_state -> Prepare_state -> Run_state -> Halt_state
//                                                     |_______|             
//      


#define		CURRENT_BUFFER_LENGTH						10

/* Exported types ------------------------------------------------------------*/
/*ϵͳ״̬����-------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		u8_cur_state;
	uint8_t		u8_pre_state;
}sys_state;



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
	float			f_max_current;						//��������A
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
	/*�����ɼ�ԭʼֵ*/
	uint16_t		ADC_DMA_buf[4];						//dma���ݴ��buf��ԭʼ���ݣ�ÿ�����ݲɼ�4����ƽ��

	uint16_t		u16_uvw_current[3];					//UVW�����
	
	/*����������ƫ��*/
	uint16_t		u16_uvw_curr_bias[3];				//UVW���������ƫ��

	/*����������ֵ�˲���*/
	uint16_t		history_data[3][CURRENT_BUFFER_LENGTH];	//��ŵ���ֵ��ʷ

	uint16_t		u16_uvw_sum[3];						//�������ֵ���

	uint8_t			filter_index_uvw[3];				//����ָ�����ֵbuf��λ��

	uint8_t			uvw_buffer_used[3];					//buffer�д�ŵ�������

	/*ʵ��ʹ�õĵ���ֵ��ת��Ϊ����������*/
	float			f_adc_UVW_I[3];
	
	/*ĸ�ߵ�ѹ�ɼ�*/
	float			f_adc_bus_voltage;
	uint16_t		u16_power_voltage;					//��Դ��ѹ	
	
	/*��ǰ����ֵ*/
	uint8_t			u8_hall_state;						//����ֵ
	
	/*����������*/
	int32_t			i32_encoder_last_read;				//ǰһ�δӶ�ʱ��ֱ�Ӷ�ȡ�����ݣ�����ʱ		TIM->cnt��16λ�Ĵ���
	int32_t			i32_encoder_curr_read;				//��ǰ�Ӷ�ʱ��ֱ�Ӷ�ȡ�����ݣ�����ʱ
	uint32_t		u32_pulse_width;					//�������������ʱ������ʱ����
	float 			f_motor_cal_speed;					//�����õĵ��ת�٣�rps
	float			f_shaft_cal_speed;					//�����õļ��ٺ����ת�٣�rps
	int64_t			i64_pulse_cnt;						//�ۼӵ�����λ������s
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


/*PID������------------------------------------------------------------------------------*/
typedef struct
{
	float		Ref_In;											//�ο�����
	float		Feed_Back;										//�����ź�����
	
	float		Err_T_0;										//����ƫ��ֵ
	float		Err_T_1;										//�ϴ�ƫ��ֵ
	float		Err_T_2;										//���ϴ�ƫ��ֵ
	float		Err_Dif;										//���仯
	
	float		Kp;												//��������ϵ��
	float		Ki;					 							//��������ϵ��
	float		Kd;												//΢������ϵ��
	
	float		P_Out;											//�����������ֵ
	float		I_Out;											//���ֻ������ֵ
	float		D_Out;											//΢�ֻ������ֵ
	
	float		pid_increment;									//PID��������ֵΪ-10~10
	float		Out_Pre;										//PID�������������ֵ��������������
	float		Out_Actual;										//PIDʵ�����ֵ��ֱ�Ӹ���ʱ���Ĵ�����ֵ
}PID_Struct;


/*����������------------------------------------------------------------------------------*/
typedef	struct
{
	PID_Struct	curr_pid;
	float		PW;
}current_pid_para;

/*�ٶȻ�����-------------------------------------------------------------------------------*/
typedef	struct
{
	PID_Struct	spd_pid;
//	float		err_p;
}speed_pid_para;



/*λ�û�����--------------------------------------------------------------------------------*/
typedef	struct
{
	PID_Struct	pos_pid;
//	float		err_p;
}position_pid_para;



/*������Ϣ--------------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		brake_err;
	uint8_t		power_error;
	
}error_log;

/* Exported constants --------------------------------------------------------*/
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




/* Exported macro ------------------------------------------------------------*/
#define		MAX_DUTY_CYCLE		4200-200
#define		MIN_DUTY_CYCLE		10




/* Exported functions ------------------------------------------------------- */
uint8_t	ParametersInit(void);

uint8_t	ParametersSave(void);

uint8_t	ParametersCheck(void);

void	EncoderDataInit(void);

/*GLOBAL PARAMETERS**************************************************************************/
extern	sys_state					m_sys_state;

extern	const_motor_para			m_motor;

extern	motor_runtime_para			m_motor_rt_para;

extern	motor_control_para			m_motor_ctrl;

extern	current_pid_para			m_current_pid;

extern	speed_pid_para				m_speed_pid;

extern	position_pid_para			m_position_pid;

extern	error_log					m_error;


#endif

/********************************************end of file***********************************************/










