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
/*系统状态参数-------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		u8_cur_state;
	uint8_t		u8_pre_state;
}sys_state;



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
	float			f_max_current;						//最大电流，A
	float			f_stall_torque;						//堵转转矩，mNm
	float			f_speed_constant;					//转速常数，rpm/V
	float			f_torque_constant;					//转矩常数，mNm/A
	float			f_current_constant;					//电流常数，A/mNm
	float			f_terminal_inductance;				//相电感，uH
	float			f_mechanical_time_constant;			//机械时间常数，ms
	uint8_t			u8_number_of_pole_pairs;			//磁极对数
	
}const_motor_para;


/*编码器运行参数------------------------------------------------------------------------*/
typedef struct
{		
	/*M/T法测速的标志位 */
	uint8_t			u8_M_or_T;
	/*测速符号位标志*/
	uint8_t			u8_velocity_sign;					//速度测量方向标志，0表示负数，1表示正数
	/*均值滤波器部分*/
	uint8_t			u8_speed_filter_used;				//速度buf中存储数据量
	int32_t			i32_spd_hisdata[SPEED_BUFFER_LENGTH];//存放速度差值的数组	
	int32_t			i32_spd_his_sum;	
	
	/*速度脉冲数增量部分*/
	uint16_t		u16_encoder_last_read;				//前一次从定时器直接读取的数据，高速时		TIM->cnt是16位寄存器
	uint16_t		u16_encoder_curr_read;				//当前从定时器直接读取的数据，高速时

	/*脉宽部分*/
	uint32_t		u32_pulse_width_buf[PW_BUFFER_LENGTH];
	
	/*码盘数据结果*/
	uint32_t		u32_pulse_width;					//相邻脉冲产生的时间差，低速时测量
	float 			f_motor_cal_speed;					//计算获得的电机转速，rps
	float			f_shaft_cal_speed;					//计算获得的减速后输出转速，rps
	int32_t			i32_pulse_cnt;						//累加的码盘位置数据s
}encoder_rt_para;


/*电机运转参数--------------------------------------------------------------------------*/
typedef	struct
{
	/*电流采集原始值*/
	uint16_t		ADC_DMA_buf[3];						//dma数据存放buf，原始数据，每个数据采集4次做平均
	uint16_t		u16_uvw_current;					//UVW相电流
	
	/*电流传感器偏置*/
	uint16_t		u16_uvw_curr_bias;					//UVW相电流测量偏置

	/*电流滑动均值滤波器*/
	uint16_t		history_data[CURRENT_BUFFER_LENGTH];	//存放电流值历史
	uint16_t		u16_uvw_sum;						//窗口求和值存放
	uint8_t			filter_index_uvw;					//用于指向电流值buf的位置
	uint8_t			uvw_buffer_used;					//buffer中存放的数据量

	/*实际使用的电流值，转化为浮点型数据*/
	float			f_adc_UVW_I;
	
	/*母线电压采集*/
	float			f_adc_bus_voltage;
	uint16_t		u16_power_voltage;					//电源电压	
	
	/*当前霍尔值*/
	uint8_t			u8_hall_state;						//霍尔值
	
	/*编码器数据*/
	encoder_rt_para	m_encoder;
}motor_runtime_para;

/*控制参数------------------------------------------------------------------------------*/
typedef	struct
{
	uint8_t		u8_dir;									//转动方向
	
	/*状态更新标志位*/
	uint8_t			u8_current_read_data_refreshed;		//读取反馈电流更新标志
	uint8_t			u8_current_set_data_refreshed;		//设置电流值更新
	uint8_t			u8_speed_read_data_refreshed;		//读取速度值更新标志
	uint8_t			u8_speed_set_data_refreshed;		//设置速度值更新
	uint8_t			u8_position_read_data_refreshed;	//读取位置值更新
	uint8_t			u8_position_set_data_refreshed;		//设置位置值更新
	
	uint8_t		u8_is_currloop_used;					//开启电流环标志位
	float		f_set_current;							//设置电流值，浮点型		正负确定dir和int_set_current
	int32_t		i32_set_current;						//设置电流值，无符号整形
	
	uint8_t		u8_is_speedloop_used;
	float		f_set_speed;							//设置速度值，浮点型
	int32_t		i32_set_speed;							//设置速度值，无符号整形	
	
	uint8_t		u8_is_posloop_used;
	float		f_set_position;							//设置位置值，浮点型
	int32_t		i32_set_position;						//设置位置值，无符号整形
}motor_control_para;


/*PID参数表------------------------------------------------------------------------------*/
typedef struct
{
	float		Ref_In;											//参考输入
	float		Feed_Back;										//反馈信号输入
	
	float		Err_T_0;										//最新偏差值
	float		Err_T_1;										//上次偏差值
	float		Err_T_2;										//上上次偏差值
	float		Err_Dif;										//误差变化
	
	float		Kp;												//比例增益系数
	float		Ki;					 							//积分增益系数
	float		Kd;												//微分增益系数
	
	float		P_Out;											//比例环节输出值
	float		I_Out;											//积分环节输出值
	float		D_Out;											//微分环节输出值
	
	float		pid_increment;									//PID计算后的数值为-10~10
	float		Out_Pre;										//PID计算后的理论输出值，控制量的增量
	float		Out_Actual;										//PID实际输出值，直接给定时器寄存器赋值
}PID_Struct;	


/*电流环参数------------------------------------------------------------------------------*/
typedef	struct
{
	PID_Struct	curr_pid;
	float		PW;
}current_pid_para;

/*速度环参数-------------------------------------------------------------------------------*/
typedef	struct
{
	PID_Struct	spd_pid;
//	float		err_p;
}speed_pid_para;



/*位置环参数--------------------------------------------------------------------------------*/
typedef	struct
{
	PID_Struct	pos_pid;
//	float		err_p;
}position_pid_para;



/*错误信息--------------------------------------------------------------------------------*/
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

void	CurrentFilterDataInit(void);

void	EncoderDataInit(void);

void	MotorCtrlDataInit(void);

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











