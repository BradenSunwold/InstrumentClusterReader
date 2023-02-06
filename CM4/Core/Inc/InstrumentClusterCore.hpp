
#ifndef INC_INSTRUMENTCLUSTERCORE_HPP_
#define INC_INSTRUMENTCLUSTERCORE_HPP_

#include "EventLoop.hpp"
#include <System/SystemTime.hpp>
#include <Messages/Speedometer/DisplayCommandMessage.hpp>
#include <Types/Time/Time.hpp>
#include <Components/SpeedSensor.hpp>
#include <System/Timer.hpp>
#include "main.h"

class InstrumentClusterCore
{
public:
	InstrumentClusterCore(SpeedSensor& sensorArray);

	// Main event loop
	void MainEventLoop();

private:
	// Private member variables
	SpeedSensor& mClusterSensorArray;				// Cluster dat holding wsp and trip values
	DisplayCommandMessage mDisplayCmdMessage;		// Command message to application side

};


#endif /* INC_INSTRUMENTCLUSTERCORE_HPP_ */
