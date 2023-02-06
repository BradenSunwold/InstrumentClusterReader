
#ifndef EVENTLOOP_HPP_
#define EVENTLOOP_HPP_

#include <stdint.h>
#include <stdio.h>
#include <main.h>

typedef uint32_t (*ReadExternalTimer32Bit_t)();

// Main event loop
void EventLoopCpp();
// Speed sensor GPIO interrupt callback
void UpdateSpeedTicksCpp();

// System timer test functions
void TestSystemTimer();

#ifdef __cplusplus
extern "C"
{
	void InitInterface(ReadExternalTimer32Bit_t readHwTimer, ReadExternalTimer32Bit_t readSwTimer,
							unsigned long clockFreq);
}
#else
	void InitInterface(ReadExternalTimer32Bit_t readHwTimer, ReadExternalTimer32Bit_t readSwTimer,
						unsigned long clockFreq);
#endif


#endif /* EVENTLOOP_HPP_ */
