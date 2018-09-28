/**
  ******************************************************************************
  * @file    Project/ucosii stm32f4 FPU DMP/math.c 
  * @author  Kuangjing
  * @version ucosii
  * @date    20161128
  * @brief   为attitude_algorithm.c 提供初级计算算法支持
			 包括但不限于：三角函数，绝对值，平方根，矩阵乘法，矩阵求逆等
  ******************************************************************************

  ******************************************************************************/

#include	"jingle_math.h"
#include	"math.h"
#include	"arm_math.h"
#include	"stm32f4xx.h"

	/*---------------------------------------------------------------------------
	函数名称			：Fast_Inverse_Square_Root
	参数含义			：要求平方根倒数的数
	函数功能			：求平方根倒数
					---------------------------------------
					  也可以使用自带的VSQRT指令__sqrtf()
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
	函数名称			：Fast_Square_Root
	参数含义			：要求平方根的数
	函数功能			：求平方根
					  arm_math.h中line6100也有定义
	----------------------------------------------------------------------------*/
float	Fast_Square_Root(float x)
{
	return	x * Fast_Inverse_Square_Root(x);
}



	/*---------------------------------------------------------------------------
	函数名称			：Fast_Abs
	参数含义			：要求绝对值的数
	函数功能			：求绝对值
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
	函数名称			：Fast_Arcsin
	参数含义			：
	函数功能			：快速计算反正弦
					  以±30°为分界线，分别拟合反正弦曲线，进行求解
	----------------------------------------------------------------------------*/
float	Fast_Arcsin(float x)
{
	float	y, g;
	float	num, den, result;
	uint8_t	i;														//±30°的标志位
	float	sign = 1.0f;
	
	y = x;
	/*从输入值获得结果的符号*/
	if(y < 0.0f)
	{
		y 		= 	-y;
		sign	=	-sign;
	}
	/*对输入值取完绝对值后，根据输入值的大小进行计算*/
	if(y > 0.5f)
	{
		i		=	1;
		if(y > 1.0f)												//输入值大于1，求出角度为0
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
		if(y < EPS_FLOAT)											//输入值趋近于0，可认为弧度数就是其正弦值
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
	/*根据前面标识符设置符号和象限*/
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
	函数名称			：Fast_Arctan
	参数含义			：
	函数功能			：快速计算反正切   arctan(y/x)
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
	函数名称			：Matrix_Mult_Matrix
	参数含义			：*pSrcA 矩阵A的地址
						aRow	A的行号
						aColumn	A的列号
					  *pSrcB 矩阵B的地址
						bRow	B的行号
						bColumn	B的列号
					  *pDstC 所得矩阵的地址
	函数功能			：两个的矩阵A，B相乘，所得矩阵赋给C
					  矩阵C的行数为   aRow
							 列数为   bColumn
	----------------------------------------------------------------------------*/
void	Matrix_Mult_Matrix(volatile float *pSrcA, uint8_t aRow, uint8_t aColumn,
						   volatile float *pSrcB, uint8_t bRow, uint8_t bColumn,
						   volatile float *pDstC)
{
	uint8_t	row,column;
	uint8_t	i;
	float	tmp;
	/*矩阵A的列数等于矩阵B的行数才能进行计算*/
	if(aColumn == bRow)
	{
		for(row = 0; row < aRow ; row++)							//矩阵C的行号
		{
			for(column = 0; column < bColumn ; column++)			//矩阵C的列号
			{
				/*计算新矩阵第row行，第column个元素的数值*/
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
	函数名称			：Matrix_Mult_MatrixT
	参数含义			：*pSrcA 矩阵A的地址
						aRow	A的行号
						aColumn	A的列号
					  *pSrcB 矩阵B的地址
						bRow	B的行号
						bColumn	B的列号
					  *pDstC 所得矩阵的地址
	函数功能			：矩阵A乘矩阵B的转置，所得矩阵赋给C
					  矩阵C的行数为   aRow
							 列数为   bColumn
	----------------------------------------------------------------------------*/
void	Matrix_Mult_MatrixT(volatile float *pSrcA, uint8_t aRow, uint8_t aColumn,
						    volatile float *pSrcB, uint8_t bRow, uint8_t bColumn,
							volatile float *pDstC)
{
	uint8_t row, column;
	uint8_t	i;
	float	tmp;
	/*矩阵A的列数等于矩阵B的列数才能进行计算*/
	if(aColumn == bColumn)
	{
		for(row = 0; row < aRow; row++)
		{
			for(column = 0; column < bRow; column++)
			{
				/*计算新矩阵第row行，第column个元素的数值*/
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
	函数名称			：Matrix_Plus_Matrix
	参数含义			：*pSrcA 矩阵A的地址
					  *pSrcB 矩阵B的地址
					  *pDstC 所得矩阵的地址
	函数功能			：两个矩阵A，B相加，所得矩阵赋给C
	----------------------------------------------------------------------------*/
void	Matrix_Plus_Matrix(volatile float *pSrcA, uint8_t aRow, uint8_t aColumn,
						   volatile float *pSrcB, uint8_t bRow, uint8_t bColumn,
						   volatile float *pDstC)
{
	uint8_t	i;
	uint8_t	tmp = aRow * aColumn;
	/*仅当矩阵A和B的行数列数都相等时才进行计算*/
	if((aRow == bRow)&&(aColumn == bColumn))			
	{
		for(i = 0; i < tmp; i++)
		{
			pDstC[i]	=	pSrcA[i] + pSrcB[i];
		}
	}

}

	/*---------------------------------------------------------------------------
	函数名称			：Matrix_Sub_Matrix
	参数含义			：*pSrcA 矩阵A的地址
					  *pSrcB 矩阵B的地址
					  *pDstC 所得矩阵的地址
	函数功能			：矩阵A减矩阵B，所得矩阵赋给C
	----------------------------------------------------------------------------*/
void	Matrix_Sub_Matrix (volatile float *pSrcA, uint8_t aRow, uint8_t aColumn,
						   volatile float *pSrcB, uint8_t bRow, uint8_t bColumn,
						   volatile float *pDstC)
{
	uint8_t	i;
	uint8_t	tmp = aRow * aColumn;
	/*仅当矩阵A和B的行数列数都相等时才进行计算*/
	if((aRow == bRow)&&(aColumn == bColumn))			
	{
		for(i = 0; i < tmp; i++)
		{
			pDstC[i]	=	pSrcA[i] - pSrcB[i];
		}
	}
}


	/*---------------------------------------------------------------------------
	函数名称			：Matrix_Inverse
	参数含义			：*pSrc  源矩阵的地址
					  *pDst  目标矩阵的地址
	函数功能			：求出矩阵pSrc的逆矩阵，采用高斯约旦消除法求解矩阵的逆
	----------------------------------------------------------------------------*/
void	Matrix_Inverse(float *pSrc,uint8_t dimetion, float *pDest )
{
	uint8_t	i,j,k;
	float	max, tmp;
	float	t_src[6][6];
	float	t_dst[6][6];
	/*将源矩阵放入t[][]中*/
	for(i=0;i<dimetion;i++)
	{
		for(j=0;j<dimetion;j++)
		{
			t_src[i][j] = pSrc[i*dimetion+j];
		}
	}
	/*将t_dst矩阵转换为单位矩阵*/
	for(i=0;i<dimetion;i++)
	{
		for(j=0; j<dimetion; j++)
		{
			t_dst[i][j] = (i == j)? 1.0f:0.0f;
		}
	}
	
	for(i=0;i<dimetion;i++)															//对每一列
	{
		/*寻找主元，即每列的最大值*/
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
		/*若最大元素（主元）不在对角线上，对t_src和t_dst进行行交换*/
		if( k!=i )
		{
			for(j=0; j<dimetion; j++)
			{
				tmp				=	t_src[i][j];
				t_src[i][j]		=	t_src[k][j];
				t_src[k][j]		=	tmp;
				/*t_dst跟随交换*/
				tmp				=	t_dst[i][j];
				t_dst[i][j]		=	t_dst[k][j];
				t_dst[k][j]		=	tmp;
			}
		}
		/*消除t_src中第i列除了第i行以外的所有元素*/
		tmp	=	t_src[i][i];
		for(j=0; j<dimetion; j++)
		{
			t_src[i][j]	=	t_src[i][j]/tmp;								//对角线上元素变为1
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






