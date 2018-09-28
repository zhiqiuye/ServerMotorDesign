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

#define		USE_SERVO_MOTOR_ATTRIBUTE
#define		USE_MOTOR_RT_PARA
#define		USE_MOTOR_CTRL_PARA
#define		USE_PID_CONTROL

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


#define		CURRENT_BUFFER_LENGTH						16
#define		SPEED_BUFFER_LENGTH							16
#define		PW_BUFFER_LENGTH							6

#define		M_METHORD									0
#define		T_METHORD									1


/* Exported types ------------------------------------------------------------*/
#ifdef		USE_SERVO_MOTOR_ATTRIBUTE
/*���������Թ̶�����--------------------------------------------------------------------*/
typedef	struct
{
	uint32_t	u32_line_per_loop;						//ÿȦ����
	uint8_t		u8_multiplication;						//��Ƶ��
	uint32_t	u32_pulse_per_loop;						//ÿȦ������
}encoder_attribute_para;


/*������Թ̶�����----------------------------------------------------------------------*/
typedef struct
{
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
}motor_attribute_para;



/*����̶�����--------------------------------------------------------------------------*/
typedef	struct
{
	encoder_attribute_para		m_encoder_att;			/*����������*/
	motor_attribute_para		m_motor_att;			/*�������*/
}servo_motor_attribute_para;

#endif

#ifdef	USE_MOTOR_RT_PARA
/*���������в���------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t			u8_curr_bias_ready;					//����ƫ��׼����ϱ�־λ
	/*�����ɼ�ԭʼֵ*/
	uint16_t		ADC_DMA_buf[3];						//dma���ݴ��buf��ԭʼ����
	uint16_t		adc_dma_shadow[3];					//dma���ݵ�Ӱ�ӼĴ�������ֹʹ��ʱdma���ݸı�
	int16_t			i16_uvw_current;					//UVW�����
	
	/*svpwmʹ�õ���ֵ*/
	int16_t			i16_u_current;
	int16_t			i16_v_current;
	int16_t			i16_w_current;
	
	/*����������ƫ��*/
	int16_t			i16_uvw_curr_bias;					//UVW���������ƫ��
	
	/*ʵ��ʹ�õĵ���ֵ��ת��Ϊ����������*/
	float			f_adc_UVW_I;
	float			f_adc_U_I;
	float			f_adc_V_I;
	float			f_adc_W_I;

}current_sensor_state;

/*������ֵ�˲�������----------------------------------------------------------------------*/
typedef	struct
{
	/*����������ֵ�˲���*/
	uint16_t		history_data[3][CURRENT_BUFFER_LENGTH];	//��ŵ���ֵ��ʷ
	int16_t			i16_uvw_sum;							//�������ֵ���
	int16_t			i16_u_sum;
	int16_t			i16_v_sum;
	int16_t			i16_w_sum;
	uint8_t			filter_index_uvw;						//����ָ�����ֵbuf��λ��
	uint8_t			uvw_buffer_used;						//buffer�д�ŵ�������

}current_filter_state;

/*ĸ�ߵ�ѹ����---------------------------------------------------------------------------*/
typedef	struct
{
	/*ĸ�ߵ�ѹ�ɼ�*/
	uint16_t		u16_adc_bus_voltage;					//ת��ΪmV��ĸ�ߵ�ѹֵ
	uint16_t		u16_power_voltage;						//��Դ��ѹԭʼ�ɼ�ֵ
}bus_voltage_state;


/*�������------------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t			u8_hall_state;							//����ֵ
	
}reversal_state;


/*����ʽ���������в���-------------------------------------------------------------------*/
typedef struct
{		
	uint8_t			u8_M_or_T;							//M/T�����ٵı�־λ

	uint8_t			u8_velocity_sign;					//�ٶȲ��������־��0��ʾ������1��ʾ����

	uint8_t			u8_speed_filter_used;				//�ٶ�buf�д洢������
	int32_t			i32_spd_hisdata[SPEED_BUFFER_LENGTH];//����ٶȲ�ֵ������	
	int32_t			i32_spd_his_sum;	
	
	uint16_t		u16_encoder_last_read;				//ǰһ�δӶ�ʱ��ֱ�Ӷ�ȡ�����ݣ�����ʱ		TIM->cnt��16λ�Ĵ���
	uint16_t		u16_encoder_curr_read;				//��ǰ�Ӷ�ʱ��ֱ�Ӷ�ȡ�����ݣ�����ʱ

	uint32_t		u32_pulse_width_buf[PW_BUFFER_LENGTH];	//������
	
	/*�����������ݽ��*/
	int32_t			i32_pulse_cnt;						//�ۼӵ�����λ������s
	uint32_t		u32_pulse_width;					//�������������ʱ������ʱ����
	float 			f_motor_cal_speed;					//�����õĵ��ת�٣�rps
	float			f_shaft_cal_speed;					//�����õļ��ٺ����ת�٣�rps
}inc_encoder_state;


/*����ʽ���������в���-------------------------------------------------------------------*/
typedef	struct
{
	uint8_t			u8_abs_data_refreshed;				//����ֵ���������ݸ������	0 ������	1 ��ȡ��ԭʼ����	2 ��ԭʼ������ȡ��λ��ֵ
	uint8_t			u8_abs_raw_data[4];					//����ֵ������ԭʼ������32λ
	uint8_t			u8_abs_pos_init_flag;				//��¼��ʼʱ��λ�ú���1
	uint8_t			u8_abs_history_buf_full;			//����ֵ�������������
	uint16_t		u16_abs_cnt;						//��ԭʼ���ݷ���ĽǶ�λ������
	uint16_t		history_data[4];					//�����ʷ���ݵ�buf
	float			f_abs_pos;							//���ٶ˽Ƕ�λ��
	float			f_abs_pos_init;						//��ʼʱ��ȡ�Ľ�λ��
}abs_encoder_state;

/*���ش���������------------------------------------------------------------------------*/
typedef struct
{
	uint16_t	torque_adc_buf[2];
	float		f_torque_A;
	float		f_torque_B;
}torque_sensor_state;




/*�����ת����--------------------------------------------------------------------------*/
typedef	struct
{
	reversal_state			m_reverse;							/*����ʹ�ò���*/
	
	current_sensor_state	m_current_sensor;					/*��������������*/
	
	current_filter_state	m_current_filter;					/*�����˲�������*/
	
	bus_voltage_state		m_bus_voltage;						/*ĸ�ߵ�ѹ����*/
	
	inc_encoder_state		m_inc_encoder;						/*��������������*/
	
	abs_encoder_state		m_abs_encoder;						/*����ֵ����������*/
	
	torque_sensor_state		m_torque_sensor;					/*���ش���������*/
}motor_runtime_para;

#endif


#ifdef	USE_MOTOR_CTRL_PARA
/*ϵͳ״̬����-------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		u8_cur_state;
	uint8_t		u8_pre_state;
	uint32_t	u32_node_id;									//�ڵ�ID
	uint8_t		u8_abs_encoder_used;							//�Ƿ�ʹ������������
	uint8_t		u8_use_svpwm;									//�Ƿ�ʹ��SVPWM
}sys_state;


/*���Ʋ���------------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		u8_dir;											//ת������

	/*״̬���±�־λ*/
	uint8_t		u8_current_read_data_refreshed;					//��ȡ�����������±�־
	uint8_t		u8_current_set_data_refreshed;					//���õ���ֵ����
	uint8_t		u8_speed_read_data_refreshed;					//��ȡ�ٶ�ֵ���±�־
	uint8_t		u8_speed_set_data_refreshed;					//�����ٶ�ֵ����
	uint8_t		u8_position_read_data_refreshed;				//��ȡλ��ֵ����
	uint8_t		u8_position_set_data_refreshed;					//����λ��ֵ����
	
	uint8_t		u8_is_currloop_used;							//������������־λ
	float		f_set_current;									//���õ���ֵ��������		����ȷ��dir��int_set_current
	int32_t		i32_set_current;								//���õ���ֵ���޷�������
	
	uint8_t		u8_is_speedloop_used;
	float		f_set_speed;									//�����ٶ�ֵ��������
	int32_t		i32_set_speed;									//�����ٶ�ֵ���޷�������	
	
	uint8_t		u8_is_posloop_used;
	float		f_set_position;									//����λ��ֵ��������
	int32_t		i32_set_position;								//����λ��ֵ���޷�������
	
	/*����λ����õ�ָ��*/
	float		f_feedforward_pos;								//���õ�ǰ��λ��
	float		f_feedforward_spd;								//���õ�ǰ���ٶ�ֵ
	float		f_feedforward_curr;								//���õ�ǰ������ֵ
}motion_control_state;


typedef	struct
{
	/*ϵͳ״̬����*/
	sys_state				m_sys_state;
	/*�˶���������*/
	motion_control_state	m_motion_ctrl;
}motor_control_state;

#endif

#ifdef	USE_PID_CONTROL
/*PID������------------------------------------------------------------------------------*/
typedef struct
{
	float		Ref_In;											//�ο�����
	float		Feed_Back;										//�����ź�����
	
	float		Err_T_0;										//����ƫ��ֵ
	float		Err_T_1;										//�ϴ�ƫ��ֵ
	float		Err_T_2;										//���ϴ�ƫ��ֵ
	float		Err_Dif;										//���仯
	float		Err_Sum;										//����
	
	float		Kp;												//��������ϵ��
	float		Ki;					 							//��������ϵ��
	float		Kd;												//΢������ϵ��
	
	float		P_Out;											//�����������ֵ
	float		I_Out;											//���ֻ������ֵ
	float		D_Out;											//΢�ֻ������ֵ
	
	float		pid_increment;									//PID��������ֵΪ-10~10
	float		Out_Pre;										//PID�������������ֵ��������������
	float		Out_Actual;										//PIDʵ�����ֵ��ֱ�Ӹ���ʱ���Ĵ�����ֵ
}PID_struct;	


typedef	struct
{
	PID_struct	curr;											/*����������*/
	PID_struct	spd;											/*�ٶȻ�����*/			
	PID_struct	pos;											/*λ�û�����*/
	float		PW;
}motor_pid_control;


#endif


typedef	struct
{
	/*CAN 1 FIFO 0�ϵ��շ�����*/
	CanTxMsg 	TxMessage;										//���ͱ��Ľṹ��
	uint8_t		mbox;											//CAN��������
	uint8_t		CAN_msg_num[3];									//��¼�ĸ����䱻ʹ��
	CanRxMsg 	RxMessage;										//���ձ��Ľṹ��
	uint8_t		CAN_msg_recv_flag;								//can���߽����ж�ʱ��1
}can_bus;

/*������Ϣ--------------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		brake_err;
	uint8_t		power_error;
	
}error_log;



/*��������----------С����ǰ--------------------------------------------------------------*/
typedef	union
{
	uint8_t		segment[2];
	uint16_t 	u16_data;
}U16;

typedef	union
{
	uint8_t		segment[2];
	int16_t		i16_data;
}I16;

typedef	union
{
	uint8_t		segment[4];
	uint32_t	u32_data;
}U32;

typedef	union
{
	uint8_t		segment[4];
	int32_t		i32_data;
}I32;



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
#define		MAX_DUTY_CYCLE		4190.0f
#define		MIN_DUTY_CYCLE		10.0f




/* Exported functions ------------------------------------------------------- */
uint8_t	ParametersInit(void);

void	CurrentFilterDataInit(void);

void	EncoderDataInit(void);

void	MotorCtrlDataInit(void);

void	SystemStateDataInit(void);

uint8_t	ParametersSave(void);

uint8_t	ParametersCheck(void);

void	EncoderDataInit(void);

/*GLOBAL PARAMETERS**************************************************************************/

extern	servo_motor_attribute_para	m_motor_attribute_para;

extern	motor_runtime_para			m_motor_rt_para;

extern	motor_control_state			m_motor_ctrl;

extern	motor_pid_control			m_pid;

extern	can_bus						m_can;

extern	error_log					m_error;


#endif

/********************************************end of file***********************************************/











