
#include "EventLoop.hpp"
#include <System/SystemTime.hpp>
#include <Messages/Speedometer/DisplayCommandMessage.hpp>
#include <Types/Time/Time.hpp>
#include <Components/SpeedSensor.hpp>

using namespace sys::time;
using namespace types;

// Globals for IPC communication
extern char clusterDataBuffer[12];		// Always have 12 bytes of data being sent to display

// Globals for speed sensor interrupts
extern volatile uint32_t speedTicks;
SpeedSensor clusterSensorArray;

// Globals for testing system time
TimeCount previousTime = 0;
int timeError = 0;

void EventLoopCpp()
{
	/* Test system timer */
//	TestSystemTimer();

	// Run state machine for packing messages
}

void UpdateClusterDataCpp()
{
	// Record time of interrupt and use it to calculate wheel speed
	TimeCount currentTime = SystemTime::GetTime();
	clusterSensorArray.UpdateInstrumentData(currentTime);
}

void TestSystemTimer()
{
	char dest[80];

	while(1)
	{

		HAL_Delay(5);
//		HAL_GPIO_TogglePin(TestPin_GPIO_Port, TestPin_Pin);
		TimeCount currentTime = SystemTime::GetTime();

		// Output system timer errors
		TimeCount timeDiff = currentTime - previousTime;
		if(timeDiff > 6005 || timeDiff < 5095)
		{
//			timeError++;
//			sprintf(dest, "%d", timeError);
//			TransmitErrors(dest);
//			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		}
		previousTime = currentTime;
	}
}

// All code for initializations between RiBearMBed and STM
extern "C"
{
	void InitInterface(ReadExternalTimer32Bit_t readHwTimer, ReadExternalTimer32Bit_t readSwTimer,
						unsigned long clockFreq)
	{
		SystemTime::Load(readHwTimer, readSwTimer, clockFreq);
	}
}
