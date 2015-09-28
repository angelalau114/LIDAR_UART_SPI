#include "TimeOut.h"
#include "Arduino.h"

TimeOut::TimeOut()
{
	_isTimeout = true;
	_duration = 0;
	_startTime = millis();
}

//-----------------------------------------------------------------------------------------------------------------------------

void TimeOut::start(unsigned long duration)
{
	_isTimeout = false;
	_duration = duration;
	_startTime = millis();
}

//-----------------------------------------------------------------------------------------------------------------------------

bool TimeOut::hasTimeOut()
{
	if (_isTimeout)
	{
		return true;
	}

	if (_startTime + _duration <= millis())
	{
		_isTimeout = true;
		return true;
	}
	return false;
}