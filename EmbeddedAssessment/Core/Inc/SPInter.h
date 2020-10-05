/*
 * SPInter.h
 *
 *  Created on: Oct 5, 2020
 *      Author: Training
 */

#ifndef INC_SPINTER_H_
#define INC_SPINTER_H_

#include "main.h"
#include "stm32f4xx_hal_spi.h"
void SPIfunction(SPI_HandleTypeDef hspi1,uint8_t *bufferTx);

#endif /* INC_SPINTER_H_ */
