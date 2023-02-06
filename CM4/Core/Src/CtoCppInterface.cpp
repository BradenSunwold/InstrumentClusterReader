
#include "CtoCppInterface.hpp"

extern "C"
{
	void EventLoopC()
	{
		EventLoopCpp();
	}

	void UpdateSpeedTicksC()
	{
		UpdateSpeedTicksCpp();
	}
}
