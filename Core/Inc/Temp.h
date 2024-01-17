/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Temp_H
#define __Temp_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t);
double temp_convert(uint16_t);

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#endif