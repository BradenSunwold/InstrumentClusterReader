
#ifndef INSTRUMENTCLUSTERDEFINES_HPP_
#define INSTRUMENTCLUSTERDEFINES_HPP_

#include <stdint.h>

typedef enum endian_t
{
	little = 0,
	big = 1
} endian_t;

void FloatToBytes(float wspd, endian_t endianType, char* buffer, int index);


#endif /* INSTRUMENTCLUSTERDEFINES_HPP_ */
