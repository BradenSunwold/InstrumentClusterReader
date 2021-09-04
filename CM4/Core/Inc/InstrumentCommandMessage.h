
#ifndef INSTRUMENTCOMMANDMESSAGE_HPP_
#define INSTRUMENTCOMMANDMESSAGE_HPP_

#include <InstrumentClusterDefines.hpp>

class InstrumentCommandMessage
{
	// Define structure to hold wheel speed, trip and odometer values
	struct instrumentData {
		  float wspd;
		  int trip;
		  int odom;
	};

public:
	InstrumentCommandMessage();
	InstrumentCommandMessage(float wsp, int trip, int odom);

	void Serialize(char* buffer, endian_t endianType);
	void LoadMessageData(float wspd, int trip, int odom);

	// Setters
	void setWspd(float wsp);
	void setTrip(int trip);
	void setOdom(int odom);

	// Getter
	float getWspd();
	int getTrip();
	int getOdom();

private:
	instrumentData mInstrumentData;

};



#endif /* INSTRUMENTCOMMANDMESSAGE_HPP_ */
