/*
 * UltraSonic.h
 *
 *  Created on: Oct 5, 2020
 *      Author: Training
 */

//#ifndef INC_ULTRASONIC_H_
//#define INC_ULTRASONIC_H_
#include "main.h"


typedef struct
{
	int numTicks;
	float distance;
}Ultra_Sonic;


/*
 * To calculate the distance of the obstacle
 */
uint8_t DistanceCal();

/*
 * To set the delay in Micro Seconds
 */
void usDelay(uint8_t uSec);

//#endif /* INC_ULTRASONIC_H_ */
