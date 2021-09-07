
#include <InstrumentCommandMessage.hpp>

InstrumentCommandMessage::InstrumentCommandMessage()
{
	mInstrumentData.wspd = 0;
	mInstrumentData.trip = 0;
	mInstrumentData.odom = 0;
}

InstrumentCommandMessage::InstrumentCommandMessage(float wsp, int trip, int odom)
{
	mInstrumentData.wspd = wsp;
	mInstrumentData.trip = trip;
	mInstrumentData.odom= odom;
}

void InstrumentCommandMessage::Serialize(char* buffer, endian_t endianType)
{
	FloatToBytes(mInstrumentData.wspd, endianType, buffer, 0);
	FloatToBytes(mInstrumentData.trip, endianType, buffer, 4);
	FloatToBytes(mInstrumentData.odom, endianType, buffer, 8);
}

void InstrumentCommandMessage::LoadMessageData(float wspd, int trip, int odom)
{
	mInstrumentData.wspd = wspd;
	mInstrumentData.trip = trip;
	mInstrumentData.odom = odom;
}

void InstrumentCommandMessage::setWspd(float wsp)
{
	mInstrumentData.wspd= wsp;
}

void InstrumentCommandMessage::setTrip(int trip)
{
	mInstrumentData.trip = trip;
}

void InstrumentCommandMessage::setOdom(int odom)
{
	mInstrumentData.odom = odom;
}

float InstrumentCommandMessage::getWspd()
{
	return mInstrumentData.wspd;
}

int InstrumentCommandMessage::getTrip()
{
	return mInstrumentData.trip;
}

int InstrumentCommandMessage::getOdom()
{
	return mInstrumentData.odom;
}
