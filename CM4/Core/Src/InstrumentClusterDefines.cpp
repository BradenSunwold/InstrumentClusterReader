

#include <InstrumentClusterDefines.hpp>

void FloatToBytes(float data, endian_t endianType, char* buffer, int index)
{
	int shift;

	if(endianType == little)
	{
		shift = 24;
		for(int i = 0; i < 4; i++)
		{
			buffer[index + i] = ((uint32_t)data >> shift) & 8;
			shift -= 8;
		}
	}
	else
	{
		shift = 0;
		for(int i = 0; i < 4; i++)
		{
			buffer[index + i] = ((uint32_t)data >> shift) & 8;
			shift += 8;
		}
	}
}

