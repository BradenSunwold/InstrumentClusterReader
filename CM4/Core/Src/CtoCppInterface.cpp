
#include "CtoCppInterface.hpp"

extern "C"
{
	void EventLoopC()
	{
		EventLoopCpp();
	}

	void UpdateClusterDataC()
	{
		UpdateClusterDataCpp();
	}
}
