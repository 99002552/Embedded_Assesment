/*
 * AnalogToDigital.c
 *
 *  Created on: Oct 5, 2020
 *      Author: Training
 */
#include "AnalogToDigital.h"

uint32_t ADCConverter(ADC_HandleTypeDef hadc1)
{
	uint32_t adcval=0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_OK);
	adcval = HAL_ADC_GetValue(&hadc1);
	return adcval;
}

