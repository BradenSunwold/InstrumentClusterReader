/*
 * InstrumentSensor.h
 *
 *  Created on: Jul 3, 2021
 *      Author: braden
 */

#ifndef INSTRUMENTSENSOR_H_
#define INSTRUMENTSENSOR_H_

#include <stdint.h>

void UpdateInstrumentData(uint32_t* totalTicks, float* wspd, int* odom, int* trip,
		float wheelCircum, int numSensors, uint32_t ticks, int tipDistanceInTicks, float frameTime);

#endif /* INSTRUMENTSENSOR_H_ */
