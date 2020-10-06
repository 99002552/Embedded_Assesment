/*
 * SPInter.c
 *
 *  Created on: Oct 5, 2020
 *      Author: Training
 */

#include "SPInter.h"

/*
 * @Brief description
 *
 * @Function-
 *
 * @Param1- hspi1
 *
 * @Param2- bufferTx
 *
 * @Definition- bufferTx holds the data that has to be sent to slave
 *
 * @Design by- Santhosh Kumar Kadaveru
 *
 * @Date and Time- 05/10/2020
 *
 */
void SPIfunction(SPI_HandleTypeDef hspi1,uint8_t *bufferTx)
{
	HAL_SPI_Transmit(&hspi1, bufferTx, 1, HAL_MAX_DELAY);
}
