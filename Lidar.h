#pragma once


#include "Arduino.h"
#include "LidarComs.h"
#include "Error.h"

#define C_LIDAR_MOTOCTL 3

class Lidar
{
public:
	Lidar(volatile Error* e);
	bool start();
	void stop();
	int run();
	bool isStart() { return _isConnected; }
	
	DataPacket getLastData(){ _lidarComs.getLastDataPacket(); }

	volatile DataPacket* getPrt(){ return _lidarComs.getPrt(); }

private:
	LidarComs _lidarComs;
	bool _isConnected;

	
};

