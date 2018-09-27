/**
  ******************************************************************************
  * @file    Project/ucosii stm32f4 FPU DMP/math.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20161128
  * @brief   Ϊattitude_algorithm.c �ṩ���������㷨֧��
			 �����������ڣ����Ǻ���������ֵ��ƽ����������˷������������
  ******************************************************************************

  ******************************************************************************/

#include	"jingle_math.h"
#include	"math.h"
#include	"arm_math.h"
#include	"stm32f4xx.h"

	/*---------------------------------------------------------------------------
	��������			��Fast_Inverse_Square_Root
	��������			��Ҫ��ƽ������������
	��������			����ƽ��������
					---------------------------------------
					  Ҳ����ʹ���Դ���VSQRTָ��__sqrtf()
	----------------------------------------------------------------------------*/
float	Fast_Inverse_Square_Root(float x)
{
	union	
	{
		unsigned int i; 
		float f;
	}l2f;
	
	l2f.f	=	x;
	l2f.i	=	0x5F1F1412 - (l2f.i >> 1);
	return	l2f.f*(1.69000231f - 0.714158168f * x * l2f.f * l2f.f);
}



	/*---------------------------------------------------------------------------
	��������			��Fast_Square_Root
	��������			��Ҫ��ƽ��������
	��������			����ƽ����
					  arm_math.h��line6100Ҳ�ж���
	----------------------------------------------------------------------------*/
float	Fast_Square_Root(float x)
{
	return	x * Fast_Inverse_Square_Root(x);
}



	/*---------------------------------------------------------------------------
	��������			��Fast_Abs
	��������			��Ҫ�����ֵ����
	��������			�������ֵ
	----------------------------------------------------------------------------*/
float	Fast_Abs(float x)
{
	union
	{
		unsigned	int	i;
		float			f;
	}y;
	
	y.f		=	x;
	y.i		=	y.i & 0x7FFFFFFF;
	return	(float)y.f;
}


	/*---------------------------------------------------------------------------
	��������			��Fast_Arcsin
	��������			��
	��������			�����ټ��㷴����
					  �ԡ�30��Ϊ�ֽ��ߣ��ֱ���Ϸ��������ߣ��������
	----------------------------------------------------------------------------*/
float	Fast_Arcsin(float x)
{
	float	y, g;
	float	num, den, result;
	uint8_t	i;														//��30��ı�־λ
	float	sign = 1.0f;
	
	y = x;
	/*������ֵ��ý���ķ���*/
	if(y < 0.0f)
	{
		y 		= 	-y;
		sign	=	-sign;
	}
	/*������ֵȡ�����ֵ�󣬸�������ֵ�Ĵ�С���м���*/
	if(y > 0.5f)
	{
		i		=	1;
		if(y > 1.0f)												//����ֵ����1������Ƕ�Ϊ0
		{
			result	=	0.0f;
			return	result;
		}
		g 		=	(1.0f - y) * 0.5f;
		y		=	-2.0f * Fast_Square_Root(g);
	}
	else
	{
		i		=	0;
		if(y < EPS_FLOAT)											//����ֵ������0������Ϊ����������������ֵ
		{
			result	=	y;
			if(sign < 0.0f)
				result	=	-result;
			return result;
		}
		g		=	y * y;
	}
	num 		=	((ASINP_COEF3 * g + ASINP_COEF2) * g + ASINP_COEF1) * g;
	den 		=	((g + ASINQ_COEF2) * g + ASINQ_COEF1) * g + ASINQ_COEF0;
	result		=	num / den;
	result		=	result * y + y;
	/*����ǰ���ʶ�����÷��ź�����*/
	if(i == 1)
	{
		result	=	result + _PI_2;
	}
	if(sign < 0.0f)
	{
		result	= -result;
	}
	return	result;
}




	/*---------------------------------------------------------------------------
	��������			��Fast_Arctan
	��������			��
	��������			�����ټ��㷴����   arctan(y/x)
	----------------------------------------------------------------------------*/
float	Fast_Arctan2(float y, float	x)
{	
	float f, g;
	float num, den;
	float result;
	int n;

	static const float a[4] = {0, (float)_PI_6, (float)_PI_2, (float)_PI_3};

	if (x == 0.0f)
	{
		if (y == 0.0f)
		{
			result = 0.0;
			return result;
		}

		result = _PI_2;
		
		if (y > (float)0.0)
			return result;
		
		if (y < (float)0.0)
		{
			result = -result;
			return result;
		}
	}
	n	= 0;
	num = y;
	den = x;

	if (num < (float)0.0)
		num		=	-num;
	
	if (den < (float)0.0)
		den		=	-den;
	
	if (num > den)
	{
		f		=	den;
		den		=	num;
		num		=	f;
		n 		=	2;
	}
	f = num / den;

	if (f > TWO_MINUS_ROOT3)
	{
		num 	=	f * SQRT3_MINUS_1 - 1.0f + f;
		den 	=	SQRT3 + f;
		f 		=	num / den;
		n 		=	n + 1;
	}

	g = f;
	if (g < 0.0f)
		g		=	-g;

	if (g < EPS_FLOAT)
		result	=	f;
	else
	{
		g		=	f * f;
		num		=	(ATANP_COEF1 * g + ATANP_COEF0) * g;
		den		=	(g + ATANQ_COEF1) * g + ATANQ_COEF0;
		result	=	num / den;
		result	=	result * f + f;
	}
	
	if (n > 1)
		result 	=	-result;

	result 		=	result + a[n];

	if (x < 0.0f)
		result 	=	_PI - result;
	
	if (y < 0.0f)
		result 	=	-result;

	return result;
}




	/*---------------------------------------------------------------------------
	��������			��Matrix_Mult_Matrix
	��������			��*pSrcA ����A�ĵ�ַ
						aRow	A���к�
						aColumn	A���к�
					  *pSrcB ����B�ĵ�ַ
						bRow	B���к�
						bColumn	B���к�
					  *pDstC ���þ���ĵ�ַ
	��������			�������ľ���A��B��ˣ����þ��󸳸�C
					  ����C������Ϊ   aRow
							 ����Ϊ   bColumn
	----------------------------------------------------------------------------*/
void	Matrix_Mult_Matrix(volatile float *pSrcA, uint8_t aRow, uint8_t aColumn,
						   volatile float *pSrcB, uint8_t bRow, uint8_t bColumn,
						   volatile float *pDstC)
{
	uint8_t	row,column;
	uint8_t	i;
	float	tmp;
	/*����A���������ھ���B���������ܽ��м���*/
	if(aColumn == bRow)
	{
		for(row = 0; row < aRow ; row++)							//����C���к�
		{
			for(column = 0; column < bColumn ; column++)			//����C���к�
			{
				/*�����¾����row�У���column��Ԫ�ص���ֵ*/
				tmp		=	0;
				for(i=0; i<aColumn; i++)
				{
					tmp += pSrcA[row * aColumn + i] * pSrcB[i * bColumn + column];
				}
				
				pDstC[row * bColumn + column]	=	tmp;
			}
		}
	}

}



	/*---------------------------------------------------------------------------
	��������			��Matrix_Mult_MatrixT
	��������			��*pSrcA ����A�ĵ�ַ
						aRow	A���к�
						aColumn	A���к�
					  *pSrcB ����B�ĵ�ַ
						bRow	B���к�
						bColumn	B���к�
					  *pDstC ���þ���ĵ�ַ
	��������			������A�˾���B��ת�ã����þ��󸳸�C
					  ����C������Ϊ   aRow
							 ����Ϊ   bColumn
	----------------------------------------------------------------------------*/
void	Matrix_Mult_MatrixT(volatile float *pSrcA, uint8_t aRow, uint8_t aColumn,
						    volatile float *pSrcB, uint8_t bRow, uint8_t bColumn,
							volatile float *pDstC)
{
	uint8_t row, column;
	uint8_t	i;
	float	tmp;
	/*����A���������ھ���B���������ܽ��м���*/
	if(aColumn == bColumn)
	{
		for(row = 0; row < aRow; row++)
		{
			for(column = 0; column < bRow; column++)
			{
				/*�����¾����row�У���column��Ԫ�ص���ֵ*/
				tmp = 0;
				for(i = 0; i < aColumn; i++)
				{
					tmp += pSrcA[row * aColumn + i] * pSrcB[column * bColumn + i];
				}
				pDstC[row * bRow + column] = tmp;
			}
		}
	}
}



	/*---------------------------------------------------------------------------
	��������			��Matrix_Plus_Matrix
	��������			��*pSrcA ����A�ĵ�ַ
					  *pSrcB ����B�ĵ�ַ
					  *pDstC ���þ���ĵ�ַ
	��������			����������A��B��ӣ����þ��󸳸�C
	----------------------------------------------------------------------------*/
void	Matrix_Plus_Matrix(volatile float *pSrcA, uint8_t aRow, uint8_t aColumn,
						   volatile float *pSrcB, uint8_t bRow, uint8_t bColumn,
						   volatile float *pDstC)
{
	uint8_t	i;
	uint8_t	tmp = aRow * aColumn;
	/*��������A��B���������������ʱ�Ž��м���*/
	if((aRow == bRow)&&(aColumn == bColumn))			
	{
		for(i = 0; i < tmp; i++)
		{
			pDstC[i]	=	pSrcA[i] + pSrcB[i];
		}
	}

}

	/*---------------------------------------------------------------------------
	��������			��Matrix_Sub_Matrix
	��������			��*pSrcA ����A�ĵ�ַ
					  *pSrcB ����B�ĵ�ַ
					  *pDstC ���þ���ĵ�ַ
	��������			������A������B�����þ��󸳸�C
	----------------------------------------------------------------------------*/
void	Matrix_Sub_Matrix (volatile float *pSrcA, uint8_t aRow, uint8_t aColumn,
						   volatile float *pSrcB, uint8_t bRow, uint8_t bColumn,
						   volatile float *pDstC)
{
	uint8_t	i;
	uint8_t	tmp = aRow * aColumn;
	/*��������A��B���������������ʱ�Ž��м���*/
	if((aRow == bRow)&&(aColumn == bColumn))			
	{
		for(i = 0; i < tmp; i++)
		{
			pDstC[i]	=	pSrcA[i] - pSrcB[i];
		}
	}
}


	/*---------------------------------------------------------------------------
	��������			��Matrix_Inverse
	��������			��*pSrc  Դ����ĵ�ַ
					  *pDst  Ŀ�����ĵ�ַ
	��������			���������pSrc������󣬲��ø�˹Լ�����������������
	----------------------------------------------------------------------------*/
void	Matrix_Inverse(float *pSrc,uint8_t dimetion, float *pDest )
{
	uint8_t	i,j,k;
	float	max, tmp;
	float	t_src[6][6];
	float	t_dst[6][6];
	/*��Դ�������t[][]��*/
	for(i=0;i<dimetion;i++)
	{
		for(j=0;j<dimetion;j++)
		{
			t_src[i][j] = pSrc[i*dimetion+j];
		}
	}
	/*��t_dst����ת��Ϊ��λ����*/
	for(i=0;i<dimetion;i++)
	{
		for(j=0; j<dimetion; j++)
		{
			t_dst[i][j] = (i == j)? 1.0f:0.0f;
		}
	}
	
	for(i=0;i<dimetion;i++)															//��ÿһ��
	{
		/*Ѱ����Ԫ����ÿ�е����ֵ*/
		max =	t_src[i][i];
		k	=	i;
		for(j=i+1;j<dimetion;j++)
		{
			if(Fast_Abs(t_src[j][i]) > Fast_Abs(max))
			{
				max	=	t_src[j][i];
				k	=	j;
			}
		}
		/*�����Ԫ�أ���Ԫ�����ڶԽ����ϣ���t_src��t_dst�����н���*/
		if( k!=i )
		{
			for(j=0; j<dimetion; j++)
			{
				tmp				=	t_src[i][j];
				t_src[i][j]		=	t_src[k][j];
				t_src[k][j]		=	tmp;
				/*t_dst���潻��*/
				tmp				=	t_dst[i][j];
				t_dst[i][j]		=	t_dst[k][j];
				t_dst[k][j]		=	tmp;
			}
		}
		/*����t_src�е�i�г��˵�i�����������Ԫ��*/
		tmp	=	t_src[i][i];
		for(j=0; j<dimetion; j++)
		{
			t_src[i][j]	=	t_src[i][j]/tmp;								//�Խ�����Ԫ�ر�Ϊ1
			t_dst[i][j]	=	t_dst[i][j]/tmp;
		}
		for(j=0; j<dimetion; j++)
		{
			if(j != i)
			{
				tmp		=	t_src[j][i];
				for(k=0; k<dimetion; k++)
				{
					t_src[j][k]	=	t_src[j][k] - t_src[i][k] * tmp;
					t_dst[j][k]	=	t_dst[j][k] - t_dst[i][k] * tmp;
				}
			}
		}
	}
	
	for(i = 0; i<dimetion; i++)
	{
		for(j=0; j<dimetion; j++)
		{
			pDest[i*dimetion+j] = t_dst[i][j];
		}
	}
}






