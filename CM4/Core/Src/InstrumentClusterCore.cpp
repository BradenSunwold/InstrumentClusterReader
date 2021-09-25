
#include "InstrumentClusterCore.hpp"

InstrumentClusterCore::InstrumentClusterCore(SpeedSensor& sensorArray,
													int displayUpdateRate)
											:
											 mClusterSensorArray(sensorArray),
											 mDisplayUpdateTimer(displayUpdateRate),
											 mContinuousOperation(true)
{
}

void InstrumentClusterCore::RunStateMachine()
{
	while(mContinuousOperation)
	{

	}
}
