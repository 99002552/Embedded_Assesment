/*
 * LightEmittingDiode.c
 *
 *  Created on: Oct 5, 2020
 *      Author: Training
 */
#include "LightEmittingDiode.h"

/*
 * @Brief description Indicate whether the system is ON or OFF
 *
 * @Function-
 *
 * @Param1- state
 *
 * @Definition- state = 1 -> System ON and state = 0 -> System OFF
 *
 * @Design by- Santhosh Kumar Kadaveru
 *
 * @Date and Time- 04/10/2020
 *
 */
void led_start(int state)
{
	if(state ==1)
	{
		HAL_GPIO_WritePin(LEDGREEN_GPIO_Port, LEDGREEN_Pin, GPIO_PIN_SET);

	}
	else
	{
		HAL_GPIO_WritePin(LEDGREEN_GPIO_Port, LEDGREEN_Pin, GPIO_PIN_RESET);
	}
}

/*
 * @Brief description- Turns on the LEDs when an obstacle is detected
 *
 * @Function-
 *
 * @Param1- Dist
 *
 * @Definition- state < 10 -> LED ON else LED OFF
 *
 * @Design by- Santhosh Kumar Kadaveru
 *
 * @Date and Time- 04/10/2020
 *
 */
void led_obstacle(uint8_t *Dist)
{
	if(*Dist <10)
	{
		HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(Blue_GPIO_Port, Blue_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Orange_GPIO_Port, Orange_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Blue_GPIO_Port, Blue_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Orange_GPIO_Port, Orange_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin,GPIO_PIN_RESET);
	}
}


