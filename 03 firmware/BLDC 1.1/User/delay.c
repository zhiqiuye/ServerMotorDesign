/**
  ******************************************************************************
  * @file    Project/user/delay.c 
  * @author  Kuangjing
  * @version ucosii 2.92
  * @date    20170724
  * @brief   使用systick进行延时
  ******************************************************************************
  ******************************************************************************
  */

#include	"delay.h"


	/*---------------------------------------------------------------------------
	函数名称			：delay_us
	参数含义			：uint32_t	dlyTime   --  延时微秒数
	函数功能			：利用systick进行延时，延时单位为1us，不会进行任务切换
	----------------------------------------------------------------------------*/
void	delay_us(uint32_t	dlyTime)
{
	int32_t	curCounter	=	0;
	int32_t	preTickVal	=	OS_CPU_CM4_NVIC_ST_CURRENT;
	int32_t	curTickVal	=	0;
	dlyTime	=	168000000/1000000*dlyTime;						//sysTick使用AHB时钟HCLK（168000000）或者其8分频
	for(;;)
	{
		curTickVal		=	OS_CPU_CM4_NVIC_ST_CURRENT;
		if(curTickVal<preTickVal)								//sysTick计数器是递减的，正常未重载寄存器，cur要小于pre
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















