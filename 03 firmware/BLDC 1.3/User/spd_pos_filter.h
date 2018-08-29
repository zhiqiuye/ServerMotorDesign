/**
  ******************************************************************************
  * @file    Project/user/spd_pos_filter.h 
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
//卡尔曼滤波器参数结构体
typedef	struct
{
	float	for_X[2];					//对速度位置的预测量,X1为角度预测值，X2为速度预测值
	float	X_k[2];						//当前拍计算结果
	float	X_k_1[2];					//前一拍计算结果	
	
	float	for_Pk[4];					//k时刻误差的协方差预测值
	float	P_k_1[4];					//k-1时刻计算的误差协方差矩阵	
	float	ctrl_a;						//控制量，加速度
	float	Kk[2];						//k时刻计算的卡尔曼增益
	//常量
	float	A[4];
	float	B[2];
	float	Q[4];						//系统过程噪声，常量
	float	R[4];						//系统测量噪声
	
}KF_filter_struct;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern	KF_filter_struct	m_KF;				//卡尔曼滤波器结构体
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


void	KF_Filter_Init(KF_filter_struct * KF_struct);

void	KF_Filter(int32_t * input_pos,float * input_spd, int32_t * output_pos, float * output_spd );

void	Read_IncEncoder(void);

void	Requir_AbsEncoder(void);

void	Read_AbsEncoder(void);







/*****************************END OF FILE************************************/

