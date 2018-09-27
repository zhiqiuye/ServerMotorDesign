/**
  ******************************************************************************
  * @file    Project/user/delay.c 
  * @author  Kuangjing
  * @version ucosii 2.92
  * @date    20170724
  * @brief   ʹ��systick������ʱ
  ******************************************************************************
  ******************************************************************************
  */

#include	"delay.h"


	/*---------------------------------------------------------------------------
	��������			��delay_us
	��������			��uint32_t	dlyTime   --  ��ʱ΢����
	��������			������systick������ʱ����ʱ��λΪ1us��������������л�
	----------------------------------------------------------------------------*/
void	delay_us(uint32_t	dlyTime)
{
	int32_t	curCounter	=	0;
	int32_t	preTickVal	=	OS_CPU_CM4_NVIC_ST_CURRENT;
	int32_t	curTickVal	=	0;
	dlyTime	=	168000000/1000000*dlyTime;						//sysTickʹ��AHBʱ��HCLK��168000000��������8��Ƶ
	for(;;)
	{
		curTickVal		=	OS_CPU_CM4_NVIC_ST_CURRENT;
		if(curTickVal<preTickVal)								//sysTick�������ǵݼ��ģ�����δ���ؼĴ�����curҪС��pre
		{
			curCounter	=	curCounter+preTickVal-curTickVal;
		}
		else
		{
			curCounter	=	curCounter+preTickVal + OS_CPU_CM4_NVIC_ST_RELOAD - curTickVal;
		}
		preTickVal	=	curTickVal;
		if(curCounter>=dlyTime)
			return;
	}
}















