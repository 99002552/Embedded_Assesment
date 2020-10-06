/*
 * AnalogToDigital.c
 *
 *  Created on: Oct 5, 2020
 *      Author: Training
 */
#include "AnalogToDigital.h"

/*
 * @Brief description Analog to digital converter code
 * @Function-
 *
 * @Param1- hadc1 of structure type ADC_HandleTypeDef
 *
 * @return 32-bit digitally converted value
 *
 * @Definition-
 *
 * @Design by- Santhosh Kumar Kadaveru
 *
 * @Date and Time- 04/10/2020
 *
 */
uint32_t ADCConverter(ADC_HandleTypeDef hadc1)
{
	uint32_t adcval=0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_OK);
	adcval = HAL_ADC_GetValue(&hadc1);
	return adcval;
}

