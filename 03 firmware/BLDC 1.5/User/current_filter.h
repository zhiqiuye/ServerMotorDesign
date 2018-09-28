/**
  ******************************************************************************
  * @file    Project/user/current_filter.h
  * @author  Kuangjing
  * @version ucosii
  * @date    20180808
  * @brief   电流滤波方法
  ******************************************************************************
  ******************************************************************************
  */

#ifndef		__CURRENT_FILTER_H
#define		__CURRENT_FILTER_H

#include	"stm32f4xx.h"
#include	"global_parameters.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */

void	Current_Filter_Init(void);

void	Current_Average_X8_Filter(current_sensor_state * m_parg);


#endif




/*********************************** END OF FILE*****************************************/

