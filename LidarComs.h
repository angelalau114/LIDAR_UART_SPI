#pragma once


#include "Error.h"

#define E_PACKET_CORRUPTION 1
#define E_BUFFER_OVERFLOW 2
#define C_LIDAR_SERIAL Serial

#define RPLIDAR_CMD_GET_DEVICE_INFO 0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH 0x52
#define RPLIDAR_CMD_SCAN 0x20
#define RPLIDAR_CMD_FORCE_SCAN 0x21
#define RPLIDAR_CMD_RESET 0x40
#define RPLIDAR_CMD_STOP 0x25

enum RequestType{None = 4, Scan = 0, Force_Scan = 1, Get_Info = 2, Get_Health = 3};
union InfoPacket
{
	uint8_t rawData[20];
	struct Payload
	{
		uint8_t model;
		uint16_t firmware_version;
		uint8_t hardware_version;
		uint8_t serialnum[16];
	}data;	
};

union HealthPacket
{
	uint8_t rawData[3];
	struct Payload
	{
		uint8_t status;
		uint16_t error_code;
	} data;
};

union DataPacket
{
	uint8_t rawData[5];
	struct Payload
	{
		uint8_t sync_quality;
		uint16_t angle;
		uint16_t distance;
	} data;
};

class LidarComs
{
public:
	LidarComs(volatile Error* e);
	void sendRequest(uint8_t packet);
	bool run();
	bool getIsDone(){ return (!(_dataCount == 0)); }
	InfoPacket getLastInfoPacket(){ return _lastInfoPacket; }
	HealthPacket getLastHealthPacket(){ return _lastHealthPacket; }
	DataPacket getLastDataPacket(){ _dataCount--; return _lastDataPacket; }
	int getDataCount(){ return _dataCount; }

	volatile DataPacket* getPrt(){ return _lastPrt; }

private:
	volatile Error* _e;
	RequestType _requestType;
	bool _descriptorReceived;
	int _readCount;
	InfoPacket _lastInfoPacket;
	HealthPacket _lastHealthPacket;
	DataPacket _lastDataPacket;
	DataPacket _buildDataPacket;

	volatile DataPacket* _lastPrt;
	int _dataCount;
	
	bool processIncomingData();

};

