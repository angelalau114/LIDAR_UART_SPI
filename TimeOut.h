#pragma once

/**
* \brief	A Timer to detect timeout events
*/
class TimeOut
{
public:

	/**
	* \brief	Default constructor.
	*
	* \author	Nicholas
	* \date	1/08/2015
	*/
	TimeOut();

	/**
	* \brief	Start a new timout measure
	*
	* \author	Nicholas
	* \date	1/08/2015
	*
	* \param	duration	The time to wait befor a timeout event
	*/
	void start(unsigned long duration);

	/**
	* \fn	bool Error::isError();
	*
	* \brief	Had more than duration time pasts since the call of start
	*
	* \author	Nicholas
	* \date	1/08/2015
	*
	* \return	true if a timeout occured
	*/
	bool hasTimeOut();

private:

	/** \brief	The system time when the timout measure started */
	volatile unsigned long _startTime;

	/** \brief	THe duration befor a timeout event */
	volatile unsigned long _duration;

	/** \brief	Has the time exceded the duration? */
	bool _isTimeout;

};

