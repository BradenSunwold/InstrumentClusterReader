
#include "EventLoop.hpp"
#include <System/SystemTime.hpp>
#include <Types/Time.hpp>

using namespace sys::time;
using namespace types;

TimeCount previousTime = 0;
int timeError = 0;

void EventLoopCpp()
{
	// Test system timer
	TestSystemTimer();
}

void TestSystemTimer()
{
	char dest[80];

	while(1)
	{

		HAL_Delay(5);
		HAL_GPIO_TogglePin(TestPin_GPIO_Port, TestPin_Pin);
		TimeCount currentTime = SystemTime::GetTime();

		// Output system timer errors
		TimeCount timeDiff = currentTime - previousTime;
		if(timeDiff > 6005 || timeDiff < 5095)
		{
//			timeError++;
//			sprintf(dest, "%d", timeError);
//			TransmitErrors(dest);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
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
