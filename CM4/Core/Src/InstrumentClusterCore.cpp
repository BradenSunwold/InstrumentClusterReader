
#include "InstrumentClusterCore.hpp"

using namespace sys::time;

struct instrumentData {
	  float wspd;
	  int trip;
	  int odom;
} instData;

InstrumentClusterCore::InstrumentClusterCore(SpeedSensor& sensorArray)
										:
										mClusterSensorArray(sensorArray)
{
	MainEventLoop();	// Immediatly call into main event loop
}

void InstrumentClusterCore::MainEventLoop()
{
	char clusterData[50];

	// debug
//	float wspdTest = 0.0;
//	int tripTest = 0.0;
//	int odomTest = 0.0;

	while(true)
	{
		// If fresh data is ready to be sent to application side
		if(mClusterSensorArray.GetDataRdy())
		{
			openAmpPollForMessages(); // Poll for incoming commands from A-7 core

			// Load fresh cluster data into serial buffer
			mDisplayCmdMessage.LoadMessageData(mClusterSensorArray.GetWspd(), mClusterSensorArray.GetTrip(),
																				mClusterSensorArray.GetOdom());

			mDisplayCmdMessage.Serialize(clusterData, types::eForward);

//			// Debug byte array
//			for(int i = 0; i < 12; i++)
//			{
//				char nextByte = clusterData[i];		// Verify we correctly serialized things
//				int j = 0;
//				j++;
//			}

			// Send cluster data through IPC
			IpcTransmit(clusterData, 12);

			mClusterSensorArray.ClearDataRdy();	// Clear data ready
		}
	}
}


















