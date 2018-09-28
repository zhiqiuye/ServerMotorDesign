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

#define		USE_SERVO_MOTOR_ATTRIBUTE
#define		USE_MOTOR_RT_PARA
#define		USE_MOTOR_CTRL_PARA
#define		USE_PID_CONTROL

/* Private macro -------------------------------------------------------------*/


//encoder
#define		LINES_PER_LOOP								4096
#define		MULTIPLICATION								4

//bldc
#define		V_bus										24				//额定输入电压
#define		N_Rated										2450			//额定功率下的速度，rpm
#define		Ip											38.0f			//峰值电流，A
#define		Ic											10.1f			//连续电流，A
#define		Tc											0.706f			//连续失速转矩，Nm
#define		Tp											2.56f			//峰值失速转矩，Nm
#define		Kt_hot										0.074f			//155℃转矩灵敏度，Nm/A
#define		Kt_cold										0.081f			//25℃转矩灵敏度，Nm/A
#define		Kb_hot										7.79f			//反电动势常数，V/krpm
#define		Kb_cold										8.57f			//反电动势常数，V/krpm
#define		P											12				//计数
#define		Lm											0.18			//电感，mH

//状态定义
#define		Idle_state									0				//上电初期，外设未准备完毕，在此状态
#define		Run_state									1				//正常运行状态
#define		Prepare_state								2				//上电后，各模块初始化完毕，进入此状态
#define		Halt_state									3				//正常运行状态进入停止
//        |-----------上电                                                           
//        |           |------外设初始化未完毕                                                
//        |           |                |---------外设初始化完毕                                
//        |           |                |            |---------正常运转状态                    
//        |           |                |            |             |---------暂停阶段       
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
/*编码器属性固定参数--------------------------------------------------------------------*/
typedef	struct
{
	uint32_t	u32_line_per_loop;						//每圈线数
	uint8_t		u8_multiplication;						//倍频数
	uint32_t	u32_pulse_per_loop;						//每圈脉冲数
}encoder_attribute_para;


/*电机属性固定参数----------------------------------------------------------------------*/
typedef struct
{
	uint32_t		u32_maxspeed;						//最大速度，rpm
	float			f_no_load_speed;					//空载转速，rpm
	float			f_no_load_current;					//空载电流，A
	float			f_max_current;						//最大电流，A
	float			f_stall_torque;						//堵转转矩，mNm
	float			f_speed_constant;					//转速常数，rpm/V
	float			f_torque_constant;					//转矩常数，mNm/A
	float			f_current_constant;					//电流常数，A/mNm
	float			f_terminal_inductance;				//相电感，uH
	float			f_mechanical_time_constant;			//机械时间常数，ms
	uint8_t			u8_number_of_pole_pairs;			//磁极对数	
}motor_attribute_para;



/*电机固定参数--------------------------------------------------------------------------*/
typedef	struct
{
	encoder_attribute_para		m_encoder_att;			/*编码器数据*/
	motor_attribute_para		m_motor_att;			/*电机参数*/
}servo_motor_attribute_para;

#endif

#ifdef	USE_MOTOR_RT_PARA
/*电流环运行参数------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t			u8_curr_bias_ready;					//电流偏置准备完毕标志位
	/*电流采集原始值*/
	uint16_t		ADC_DMA_buf[3];						//dma数据存放buf，原始数据
	uint16_t		adc_dma_shadow[3];					//dma数据的影子寄存器，防止使用时dma数据改变
	int16_t			i16_uvw_current;					//UVW相电流
	
	/*svpwm使用电流值*/
	int16_t			i16_u_current;
	int16_t			i16_v_current;
	int16_t			i16_w_current;
	
	/*电流传感器偏置*/
	int16_t			i16_uvw_curr_bias;					//UVW相电流测量偏置
	
	/*实际使用的电流值，转化为浮点型数据*/
	float			f_adc_UVW_I;
	float			f_adc_U_I;
	float			f_adc_V_I;
	float			f_adc_W_I;

}current_sensor_state;

/*电流均值滤波器参数----------------------------------------------------------------------*/
typedef	struct
{
	/*电流滑动均值滤波器*/
	uint16_t		history_data[3][CURRENT_BUFFER_LENGTH];	//存放电流值历史
	int16_t			i16_uvw_sum;							//窗口求和值存放
	int16_t			i16_u_sum;
	int16_t			i16_v_sum;
	int16_t			i16_w_sum;
	uint8_t			filter_index_uvw;						//用于指向电流值buf的位置
	uint8_t			uvw_buffer_used;						//buffer中存放的数据量

}current_filter_state;

/*母线电压参数---------------------------------------------------------------------------*/
typedef	struct
{
	/*母线电压采集*/
	uint16_t		u16_adc_bus_voltage;					//转化为mV的母线电压值
	uint16_t		u16_power_voltage;						//电源电压原始采集值
}bus_voltage_state;


/*换向参数------------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t			u8_hall_state;							//霍尔值
	
}reversal_state;


/*增量式编码器运行参数-------------------------------------------------------------------*/
typedef struct
{		
	uint8_t			u8_M_or_T;							//M/T法测速的标志位

	uint8_t			u8_velocity_sign;					//速度测量方向标志，0表示负数，1表示正数

	uint8_t			u8_speed_filter_used;				//速度buf中存储数据量
	int32_t			i32_spd_hisdata[SPEED_BUFFER_LENGTH];//存放速度差值的数组	
	int32_t			i32_spd_his_sum;	
	
	uint16_t		u16_encoder_last_read;				//前一次从定时器直接读取的数据，高速时		TIM->cnt是16位寄存器
	uint16_t		u16_encoder_curr_read;				//当前从定时器直接读取的数据，高速时

	uint32_t		u32_pulse_width_buf[PW_BUFFER_LENGTH];	//脉宽部分
	
	/*增量码盘数据结果*/
	int32_t			i32_pulse_cnt;						//累加的码盘位置数据s
	uint32_t		u32_pulse_width;					//相邻脉冲产生的时间差，低速时测量
	float 			f_motor_cal_speed;					//计算获得的电机转速，rps
	float			f_shaft_cal_speed;					//计算获得的减速后输出转速，rps
}inc_encoder_state;


/*绝对式编码器运行参数-------------------------------------------------------------------*/
typedef	struct
{
	uint8_t			u8_abs_data_refreshed;				//绝对值编码器数据更新完毕	0 无数据	1 获取到原始数据	2 从原始数据提取到位置值
	uint8_t			u8_abs_raw_data[4];					//绝对值编码器原始读数，32位
	uint8_t			u8_abs_pos_init_flag;				//记录初始时角位置后置1
	uint8_t			u8_abs_history_buf_full;			//绝对值编码器缓存放满
	uint16_t		u16_abs_cnt;						//从原始数据分离的角度位置数据
	uint16_t		history_data[4];					//存放历史数据的buf
	float			f_abs_pos;							//低速端角度位置
	float			f_abs_pos_init;						//初始时获取的角位置
}abs_encoder_state;

/*力矩传感器参数------------------------------------------------------------------------*/
typedef struct
{
	uint16_t	torque_adc_buf[2];
	float		f_torque_A;
	float		f_torque_B;
}torque_sensor_state;




/*电机运转参数--------------------------------------------------------------------------*/
typedef	struct
{
	reversal_state			m_reverse;							/*换向使用参数*/
	
	current_sensor_state	m_current_sensor;					/*电流传感器数据*/
	
	current_filter_state	m_current_filter;					/*电流滤波器数据*/
	
	bus_voltage_state		m_bus_voltage;						/*母线电压数据*/
	
	inc_encoder_state		m_inc_encoder;						/*增量编码器数据*/
	
	abs_encoder_state		m_abs_encoder;						/*绝对值编码器数据*/
	
	torque_sensor_state		m_torque_sensor;					/*力矩传感器数据*/
}motor_runtime_para;

#endif


#ifdef	USE_MOTOR_CTRL_PARA
/*系统状态参数-------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		u8_cur_state;
	uint8_t		u8_pre_state;
	uint32_t	u32_node_id;									//节点ID
	uint8_t		u8_abs_encoder_used;							//是否使用增量编码器
	uint8_t		u8_use_svpwm;									//是否使用SVPWM
}sys_state;


/*控制参数------------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		u8_dir;											//转动方向

	/*状态更新标志位*/
	uint8_t		u8_current_read_data_refreshed;					//读取反馈电流更新标志
	uint8_t		u8_current_set_data_refreshed;					//设置电流值更新
	uint8_t		u8_speed_read_data_refreshed;					//读取速度值更新标志
	uint8_t		u8_speed_set_data_refreshed;					//设置速度值更新
	uint8_t		u8_position_read_data_refreshed;				//读取位置值更新
	uint8_t		u8_position_set_data_refreshed;					//设置位置值更新
	
	uint8_t		u8_is_currloop_used;							//开启电流环标志位
	float		f_set_current;									//设置电流值，浮点型		正负确定dir和int_set_current
	int32_t		i32_set_current;								//设置电流值，无符号整形
	
	uint8_t		u8_is_speedloop_used;
	float		f_set_speed;									//设置速度值，浮点型
	int32_t		i32_set_speed;									//设置速度值，无符号整形	
	
	uint8_t		u8_is_posloop_used;
	float		f_set_position;									//设置位置值，浮点型
	int32_t		i32_set_position;								//设置位置值，无符号整形
	
	/*从上位机获得的指令*/
	float		f_feedforward_pos;								//设置的前馈位置
	float		f_feedforward_spd;								//设置的前馈速度值
	float		f_feedforward_curr;								//设置的前馈电流值
}motion_control_state;


typedef	struct
{
	/*系统状态数据*/
	sys_state				m_sys_state;
	/*运动控制数据*/
	motion_control_state	m_motion_ctrl;
}motor_control_state;

#endif

#ifdef	USE_PID_CONTROL
/*PID参数表------------------------------------------------------------------------------*/
typedef struct
{
	float		Ref_In;											//参考输入
	float		Feed_Back;										//反馈信号输入
	
	float		Err_T_0;										//最新偏差值
	float		Err_T_1;										//上次偏差值
	float		Err_T_2;										//上上次偏差值
	float		Err_Dif;										//误差变化
	float		Err_Sum;										//误差和
	
	float		Kp;												//比例增益系数
	float		Ki;					 							//积分增益系数
	float		Kd;												//微分增益系数
	
	float		P_Out;											//比例环节输出值
	float		I_Out;											//积分环节输出值
	float		D_Out;											//微分环节输出值
	
	float		pid_increment;									//PID计算后的数值为-10~10
	float		Out_Pre;										//PID计算后的理论输出值，控制量的增量
	float		Out_Actual;										//PID实际输出值，直接给定时器寄存器赋值
}PID_struct;	


typedef	struct
{
	PID_struct	curr;											/*电流环参数*/
	PID_struct	spd;											/*速度环参数*/			
	PID_struct	pos;											/*位置环参数*/
	float		PW;
}motor_pid_control;


#endif


typedef	struct
{
	/*CAN 1 FIFO 0上的收发报文*/
	CanTxMsg 	TxMessage;										//发送报文结构体
	uint8_t		mbox;											//CAN发送邮箱
	uint8_t		CAN_msg_num[3];									//记录哪个邮箱被使用
	CanRxMsg 	RxMessage;										//接收报文结构体
	uint8_t		CAN_msg_recv_flag;								//can总线接收中断时置1
}can_bus;

/*错误信息--------------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		brake_err;
	uint8_t		power_error;
	
}error_log;



/*数据整合----------小端在前--------------------------------------------------------------*/
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











