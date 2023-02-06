
#ifndef INC_CTOCPPINTERFACE_HPP_
#define INC_CTOCPPINTERFACE_HPP_

#include <EventLoop.hpp>

#ifdef __cplusplus
extern "C"
{
	// Interface function call to main cpp application loop
	void EventLoopC();
	// Interface function call to speed sensor GPIO interrupt callback
	void UpdateSpeedTicksC();
}
#else
	void EventLoopC();
	void UpdateSpeedTicksC();
#endif

#endif /* INC_CTOCPPINTERFACE_HPP_ */
