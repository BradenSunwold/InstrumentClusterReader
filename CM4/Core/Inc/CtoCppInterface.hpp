
#ifndef INC_CTOCPPINTERFACE_HPP_
#define INC_CTOCPPINTERFACE_HPP_

#include <EventLoop.hpp>

#ifdef __cplusplus
extern "C"
{
	void EventLoopC();
}
#else
	void EventLoopC();
#endif

#endif /* INC_CTOCPPINTERFACE_HPP_ */
