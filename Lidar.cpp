#include "Lidar.h"
#include "TimeOut.h"


Lidar::Lidar(volatile Error* e) :_lidarComs(e)
{
	_isConnected = false;
}

bool Lidar::start()
{
	// Activate motor
	pinMode(C_LIDAR_MOTOCTL, OUTPUT);
	analogWrite(C_LIDAR_MOTOCTL, 255);

	// Reset
	_lidarComs.sendRequest(RPLIDAR_CMD_RESET);
	_lidarComs.run();

	//Empty buffer (extra data that was sent before the reset)
	delay(1000);
	while (C_LIDAR_SERIAL.available())
	{
		char t = C_LIDAR_SERIAL.read();

	}

	// Check device health
	TimeOut timeOut;
	timeOut.start(1000);
	_lidarComs.sendRequest(RPLIDAR_CMD_GET_DEVICE_HEALTH);
	while (true)
	{
		_lidarComs.run();
		if (_lidarComs.getIsDone())
		{
			HealthPacket theHealthPacket = _lidarComs.getLastHealthPacket();
			if (theHealthPacket.data.error_code != 0 || theHealthPacket.data.status != 0)
			{
				analogWrite(C_LIDAR_MOTOCTL, 0);
				_isConnected = false;
				return false;
			}

			break;
		}
		if (timeOut.hasTimeOut())
		{
			analogWrite(C_LIDAR_MOTOCTL, 0);
			_isConnected = false;
			return false;
		}
	}

	// Check device info
	timeOut.start(1000);
	_lidarComs.sendRequest(RPLIDAR_CMD_GET_DEVICE_INFO);
	while (true)
	{
		_lidarComs.run();
		if (_lidarComs.getIsDone())
		{
			InfoPacket theInfoPacket = _lidarComs.getLastInfoPacket();
			if (theInfoPacket.data.firmware_version != 271 || theInfoPacket.data.hardware_version != 0 || theInfoPacket.data.model != 0)
			{
				analogWrite(C_LIDAR_MOTOCTL, 0);
				_isConnected = false;
				return false;
			}
			break;
		}
		if (timeOut.hasTimeOut())
		{
			analogWrite(C_LIDAR_MOTOCTL, 0);
			_isConnected = false;
			return false;
		}
	}
	// Lidar checks completed, start scan
	_lidarComs.sendRequest(RPLIDAR_CMD_SCAN);
	_isConnected = true;
	return true;
}

void Lidar::stop()
{
	_lidarComs.sendRequest(RPLIDAR_CMD_STOP);
	analogWrite(C_LIDAR_MOTOCTL, 0);

	_isConnected = false;
}


int Lidar::run()
{
	_lidarComs.run();
	if (_lidarComs.getIsDone())
	{
		return 1;
	}
	return 0;
}

