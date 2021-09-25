
#ifndef INC_INSTRUMENTCLUSTERCORE_HPP_
#define INC_INSTRUMENTCLUSTERCORE_HPP_

#include "EventLoop.hpp"
#include <System/SystemTime.hpp>
#include <Messages/Speedometer/DisplayCommandMessage.hpp>
#include <Types/Time/Time.hpp>
#include <Components/SpeedSensor.hpp>

class InstrumentClusterCore
{
public:
	InstrumentClusterCore(SpeedSensor& sensorArray, int displayUpdateRate);

	// Main state machine function
	void RunStateMachine();

private:
	// Private member variables
	SpeedSensor& mClusterSensorArray;
	int mDisplayUpdateTimer;
	DisplayCommandMessage mDisplayCmdMessage;
	bool mContinuousOperation;

	// Private member functions
};


#endif /* INC_INSTRUMENTCLUSTERCORE_HPP_ */
