/*
 * UltraSonic.c
 *
 *  Created on: Oct 5, 2020
 *      Author: Training
 */

#include "UltraSonic.h"

/*
 *
 * @Function- DistanceCal()
 * @Brief description - This function is responsible for calculating the distance at which the obstacle is present.
 *
 * @Definition-
 *
 * @Design by- Santhosh Kumar Kadaveru
 *
 * @Date and Time- 05/10/2020  8:12PM
 *
 */
uint8_t DistanceCal()
{
	Ultra_Sonic *pUS;
	//uint8_t numTicks = 0;
	//float distance = 0;
	uint8_t dis=0;
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	usDelay(2);

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin)==GPIO_PIN_RESET);
	pUS->numTicks=0;
	pUS->distance=0;
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin)==GPIO_PIN_SET)
	{
		(pUS->numTicks)++;
		usDelay(1);
	};

	pUS->distance = (((pUS->numTicks)+0.0f)*0.034/2) *2.54;
	dis = (uint8_t)(pUS->distance);
	return dis;
}

/*
 *
 * @Function- usDelay()
 * @Brief description - This function is responsible for providing micro second delay.
 *
 * @param1 - uSec
 *
 * @Definition-
 *
 * @Design by- Santhosh Kumar Kadaveru
 *
 * @Date and Time- 05/10/2020  8:20PM
 *
 */
void usDelay(uint8_t uSec)
{
	for(int i=0;i<uSec;i++);
}
