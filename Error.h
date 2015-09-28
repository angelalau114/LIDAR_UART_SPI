#pragma once

#include "Arduino.h"
#define Throw_Check(toReturn)	if(_e->isError()){return toReturn;}
#define Could_Throw(function, toReturn)	function;if(_e->isError()){return toReturn;}

#define C_ERROR_BUFFER 2

/**
 * \class	Error
 *
 * \brief	A collection of error events (used in place of exceptions)
 *
 * \author	Nicholas
 * \date	1/08/2015
 */
class Error
{
public:

	/**
	 * \fn	Error::Error();
	 *
	 * \brief	Default constructor.
	 *
	 * \author	Nicholas
	 * \date	1/08/2015
	 */
	Error();

	void reset() volatile
	{
		_errors = 0;
	}

	/**
	 * \fn	void Error::add(int code, String info);
	 *
	 * \brief	Adds a error event (nothing is added if more than C_ERROR_BUFFER have been added befor)
	 *
	 * \author	Nicholas
	 * \date	1/08/2015
	 *
	 * \param	code	The code.
	 * \param	line	The line the error ocured on
	 */
	void add(uint8_t code, uint8_t  line) volatile;

	/**
	 * \fn	bool Error::isError();
	 *
	 * \brief	Are there any recorded error events
	 *
	 * \author	Nicholas
	 * \date	1/08/2015
	 *
	 * \return	true if error, false if not.
	 */
	bool isError() volatile;

	/**
	 * \fn	ErrorMessage* Error::getError();
	 *
	 * \brief	Gets the next error in the list (only call if isError return true)
	 *
	 * \author	Nicholas
	 * \date	1/08/2015
	 *
	 * \return	null if it fails, else the error.
	 */
	uint16_t getError() volatile;

private:

	/** \brief	The error events[ c error buffer]. */
	uint8_t _errorEvents[C_ERROR_BUFFER];

	/** \brief	THe line the error ocured on */
	uint8_t _errorLine[C_ERROR_BUFFER];

	/** \brief	The number of recorded errors */
	int _errors;
};

