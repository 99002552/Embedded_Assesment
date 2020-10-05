/*
 * SPInter.c
 *
 *  Created on: Oct 5, 2020
 *      Author: Training
 */

#include "SPInter.h"

void SPIfunction(SPI_HandleTypeDef hspi1,uint8_t *bufferTx)
{
	HAL_SPI_Transmit(&hspi1, bufferTx, 1, HAL_MAX_DELAY);
}
