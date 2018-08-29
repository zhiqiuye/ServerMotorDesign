/**
  ******************************************************************************
  * @file    Project/user/spd_pos_filter.h 
  * @author  Kuangjing
  * @version ucosii
  * @date    20180814
  * @brief   ���������ݶ�ȡ���ٶ�λ�ý���
  ******************************************************************************
  ******************************************************************************
  */

#include	"stm32f4xx.h"
#include	"stm32f4xx_tim.h"
#include	"global_parameters.h"



/* Private typedef -----------------------------------------------------------*/
//�������˲��������ṹ��
typedef	struct
{
	float	for_X[2];					//���ٶ�λ�õ�Ԥ����,X1Ϊ�Ƕ�Ԥ��ֵ��X2Ϊ�ٶ�Ԥ��ֵ
	float	X_k[2];						//��ǰ�ļ�����
	float	X_k_1[2];					//ǰһ�ļ�����	
	
	float	for_Pk[4];					//kʱ������Э����Ԥ��ֵ
	float	P_k_1[4];					//k-1ʱ�̼�������Э�������	
	float	ctrl_a;						//�����������ٶ�
	float	Kk[2];						//kʱ�̼���Ŀ���������
	//����
	float	A[4];
	float	B[2];
	float	Q[4];						//ϵͳ��������������
	float	R[4];						//ϵͳ��������
	
}KF_filter_struct;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern	KF_filter_struct	m_KF;				//�������˲����ṹ��
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


void	KF_Filter_Init(KF_filter_struct * KF_struct);

void	KF_Filter(int32_t * input_pos,float * input_spd, int32_t * output_pos, float * output_spd );

void	Read_IncEncoder(void);

void	Requir_AbsEncoder(void);

void	Read_AbsEncoder(void);







/*****************************END OF FILE************************************/

