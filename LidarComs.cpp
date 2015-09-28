
#include "LidarComs.h"

#define RPLIDAR_CMD_SYNC_BYTE 0xA5
#define RPLIDAR_ANS_SYNC_BYTE1 0xA5
#define RPLIDAR_ANS_SYNC_BYTE2 0x5A

const uint8_t DESCRIPTOR_RESPONSE[4][7] = { { RPLIDAR_ANS_SYNC_BYTE1, RPLIDAR_ANS_SYNC_BYTE2, 0x05, 0x00, 0x00, 0x40, 0x81 },
{ RPLIDAR_ANS_SYNC_BYTE1, RPLIDAR_ANS_SYNC_BYTE2, 0x05, 0x00, 0x00, 0x40, 0x81 },
{ RPLIDAR_ANS_SYNC_BYTE1, RPLIDAR_ANS_SYNC_BYTE2, 0x14, 0x00, 0x00, 0x00, 0x04 },
{ RPLIDAR_ANS_SYNC_BYTE1, RPLIDAR_ANS_SYNC_BYTE2, 0x03, 0x00, 0x00, 0x00, 0x06 }
};


LidarComs::LidarComs(volatile Error* e)
{
	_e = e;
	_requestType = None;
	_descriptorReceived = false;
	_dataCount = 0;
	_lastPrt = &_lastDataPacket;
}

void LidarComs::sendRequest(uint8_t packet)
{
	C_LIDAR_SERIAL.flush();
	C_LIDAR_SERIAL.write(RPLIDAR_CMD_SYNC_BYTE);
	C_LIDAR_SERIAL.write(packet);
	_dataCount = 0;
	if (packet == RPLIDAR_CMD_GET_DEVICE_INFO)
	{
		_requestType = Get_Info;
	}
	else if (packet == RPLIDAR_CMD_GET_DEVICE_HEALTH)
	{
		_requestType = Get_Health;
	}
	else if (packet == RPLIDAR_CMD_SCAN)
	{
		_requestType = Scan;
	}
	else if (packet == RPLIDAR_CMD_FORCE_SCAN)
	{
		_requestType = Force_Scan;
	}
	else
	{
		_requestType = None;
	}

	_descriptorReceived = false;
	_readCount = 0;
}

bool LidarComs::run()
{
	return processIncomingData();
}

bool LidarComs::processIncomingData()
{
	int bufferSize = C_LIDAR_SERIAL.available();
	if (bufferSize >= 62)
	{
		_e->add(E_BUFFER_OVERFLOW, __LINE__);
		return false;
	}


	for (int i = 0; i < bufferSize; i++)
	{

		uint8_t currentByte = C_LIDAR_SERIAL.read();
		if (_descriptorReceived)
		{
			switch (_requestType)
			{
			case None:
				break;
			case Scan:
				_buildDataPacket.rawData[_readCount] = currentByte;
				_readCount++;
				if (_readCount >= 5)
				{
					_readCount = 0;
					_dataCount++;
					for (int i = 0; i < 5; i++)
					{
						_lastDataPacket.rawData[i] = _buildDataPacket.rawData[i];
					}

					// dump and extra packets
					int loop = bufferSize - i - 1;
					for (int i = 0; i < (((loop / 5)) * 5); i++)
					{
						C_LIDAR_SERIAL.read();
					}

					return true;
				}
				break;
			case Force_Scan:
				_buildDataPacket.rawData[_readCount] = currentByte;
				_readCount++;
				if (_readCount >= 5)
				{
					_dataCount++;
					_readCount = 0;
					for (int i = 0; i < 5; i++)
					{
						_lastDataPacket.rawData[i] = _buildDataPacket.rawData[i];
					}

					// dump and extra packets
					int loop = bufferSize - i - 1;
					for (int i = 0; i < (((loop / 5)) * 5); i++)
					{
						C_LIDAR_SERIAL.read();
					}

					return true;
				}
				break;
			case Get_Info:
				_lastInfoPacket.rawData[_readCount] = currentByte;
				_readCount++;
				if (_readCount >= 20)
				{
					_readCount = 0;
					_dataCount = 1;
					return true;
				}
				break;
			case Get_Health:
				_lastHealthPacket.rawData[_readCount] = currentByte;
				_readCount++;
				if (_readCount >= 3)
				{
					_readCount = 0;
					_dataCount = 1;
					return true;
				}
				break;
			default:
				_e->add(E_PACKET_CORRUPTION, __LINE__);
				break;
			}
		}
		else
		{
			if (currentByte == DESCRIPTOR_RESPONSE[_requestType][_readCount])
			{
				_readCount++;
				if (_readCount >= 7)
				{
					_descriptorReceived = true;
					_readCount = 0;
					return true;
				}
			}
			else
			{
				_e->add(E_PACKET_CORRUPTION, __LINE__);
				return false;
			}
		}
	}
	return true;
}