/*
 * InstrumentSensor.c
 *
 *  Created on: Jun 30, 2021
 *      Author: braden
 */

#include "InstrumentSensor.h"

void UpdateInstrumentData(uint32_t* totalTicks, float* wspd, int* odom, int* trip,
		float wheelCircum, int numSensors, uint32_t ticks, int tipDistanceInTicks, float frameTime)
{
	// Update wheel speed
	float currDistance = (wheelCircum / numSensors) * ticks;	// calculate distance traveled since last interupt

	*wspd = currDistance / frameTime;
	*totalTicks = *totalTicks + ticks;		// Calculate the total distance traveled so far in ticks

	// Update trip mileage
	if(totalTicks >= tipDistanceInTicks)
	{
		*trip++;					// Have traveled one unit of distance so increment trip and odom
		*odom++;
		*totalTicks = 0;
	}
}
