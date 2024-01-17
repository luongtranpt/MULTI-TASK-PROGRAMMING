/*
 * Temp.c
 *
 *  Created on: Dec 13, 2023
 *      Author: Xuan Mai, Tran Luong
 */

/* Private includes ----------------------------------------------------------*/
#include "Temp.h"
#include "main.h" 

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern uint16_t adcv;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
double temp_convert(uint16_t adc1)
{  
   double temp1;
	temp1 = (double) adc1 * 100 *3.3 / (4.4 * 4096); 
	return temp1;
}