



#ifndef	__JINGLE_MATH_H
#define	__JINGLE_MATH_H

#include	"math.h"
#include	"arm_math.h"

#define	_2_PI				6.283185307179586476925286766559f
#define	_PI					3.1415926535897932384626433832795f
#define	_PI_2				1.5707963267948966192313216916398f
#define _PI_3 				1.0471975511965977461542144610932f
#define _PI_4 				0.78539816339744830961566084581988f
#define _PI_6 				0.52359877559829887307710723054658f

#define	EPS_FLOAT			+3.452669830012e-4f
#define TWO_MINUS_ROOT3		0.26794919243112270647255365849413f
#define SQRT3_MINUS_1 		0.73205080756887729352744634150587f
#define SQRT3 				1.7320508075688772935274463415059f
/*Coefficients used for atan/atan2*/
#define ATANP_COEF0 		(-1.44008344874f)
#define ATANP_COEF1 		(-7.20026848898e-1f)
#define ATANQ_COEF0 		(+4.32025038919f)
#define ATANQ_COEF1 		(+4.75222584599f)
/*Coefficients used for asin/acos*/
#define ASINP_COEF1 		(-2.7516555290596f)
#define ASINP_COEF2 		(+2.9058762374859f)
#define ASINP_COEF3 		(-5.9450144193246e-1f)
#define ASINQ_COEF0 		(-1.6509933202424e+1f)
#define ASINQ_COEF1 		(+2.4864728969164e+1f)
#define ASINQ_COEF2 		(-1.0333867072113e+1f)

float	Fast_Inverse_Square_Root(float x);

float	Fast_Square_Root(float x);

float	Fast_Abs(float x);

float	Fast_Arcsin(float x);

float	Fast_Arctan2(float y, float	x);

void	Matrix_Mult_Matrix(volatile float *pSrcA, uint8_t aRow, uint8_t aColumn,
						   volatile float *pSrcB, uint8_t bRow, uint8_t bColumn,
						   volatile float *pDstC);
						   
void	Matrix_Mult_MatrixT(volatile float *pSrcA, uint8_t aRow, uint8_t aColumn,
						    volatile float *pSrcB, uint8_t bRow, uint8_t bColumn,
							volatile float *pDstC);
							
void	Matrix_Plus_Matrix(volatile float *pSrcA, uint8_t aRow, uint8_t aColumn,
						   volatile float *pSrcB, uint8_t bRow, uint8_t bColumn,
						   volatile float *pDstC);
						   
void	Matrix_Sub_Matrix (volatile float *pSrcA, uint8_t aRow, uint8_t aColumn,
						   volatile float *pSrcB, uint8_t bRow, uint8_t bColumn,
						   volatile float *pDstC);
						   
void	Matrix_Inverse(float *pSrc,uint8_t dimetion, float *pDest );

#endif




