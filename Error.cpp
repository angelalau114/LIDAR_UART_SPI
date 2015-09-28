#include "Error.h"

Error::Error()
{
	//Serial.begin(9600);
	_errors = 0;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Error::add(uint8_t code, uint8_t  line) volatile
{
	if (_errors >= (C_ERROR_BUFFER ))
	{
		// no more room (were in big troble)
		return;
	}

	_errorEvents[_errors] = code;
	_errorLine[_errors] = line;

	_errors++;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool Error::isError() volatile
{
	if (_errors == 0)
	{
		return false;
	}
	return true;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

uint16_t Error::getError() volatile
{ 
	_errors--;

	return (_errorEvents[_errors] << 8) | _errorLine[_errors];
}
