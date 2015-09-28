
#include "Lidar.h"


// status of ongoing packet
volatile byte command = 0;
volatile int packet_byte = 0;

// commands
#define START_COMMAND	's'
#define STOP_COMMAND	't'
#define READ_COMMAND	'r'
#define ERROR_COMMAND	'e'
#define ISSTART_COMMAND 'i'

// commands to execute
bool _doStart = false;
bool _doStop = false;

// the god almighty lidar object
volatile Error _e;
Lidar _lidar(&_e);


DataPacket _last;
volatile int _readCount = 0;



void setup(void)
{
	// open serial conection to the LIDAR
	C_LIDAR_SERIAL.begin(115200);
	C_LIDAR_SERIAL.clearWriteError();
	C_LIDAR_SERIAL.flush();
	while (C_LIDAR_SERIAL.available())
	{
		C_LIDAR_SERIAL.read();
	}
	delay(500);


	// have to send on master in, *slave out*
	pinMode(MISO, OUTPUT);

	// turn on SPI in slave mode
	SPCR |= _BV(SPE);

	// turn on interrupts
	SPCR |= _BV(SPIE);

}


// SPI interrupt routine
ISR(SPI_STC_vect)
{
	// disable interupts
	cli();

	byte c = SPDR;
	SPDR = 0;

	switch (command)
	{
	case 0:	// no command? then this is the command
		command = c;
		SPDR = 0;
		// check for signle step commands
		switch (c)
		{
		case START_COMMAND:
			_doStart = true;
			break;
		case STOP_COMMAND:
			_doStop = true;
			break;
		case ISSTART_COMMAND:
			SPDR = _lidar.isStart();
			break;
		case READ_COMMAND:
			SPDR = _lidar.getPrt()->rawData[0];
			packet_byte = 1;
			break;
		case ERROR_COMMAND:
			if (_e.isError())
			{
				SPDR = _e.getError();
			}
			break;
		default:
			break;
		}
		break;

	case READ_COMMAND:
		SPDR = _lidar.getPrt()->rawData[packet_byte];
		packet_byte++;
		break;
	case 'c':
		SPDR = _readCount;
		break;
	}

	// renable interupts
	sei();
}

void loop(void)
{
	// check for single step comands
	if (_doStart == true)
	{
		_lidar.start();
		_doStart = false;
		_e.reset();
	}
	if (_doStop == true)
	{
		_lidar.stop();
		_doStop = false;
	}
	
	// check for the end of a packet, clear for the next one
	if (command != 0 && digitalRead(SS) == HIGH)
	{
		command = 0;
		packet_byte = 0;
	}

	// run the lidar
	if (_lidar.run() != 0)
	{
		if (command != 0)
		{
			_last = _lidar.getLastData();	// dose not work
			_readCount++;
		}

	}
}  // end of loop