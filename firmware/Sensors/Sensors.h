#ifndef SENSORS_H_
#define SENSORS_H_

#include <avr/io.h>
#include <stdint.h>

#define NUM_CHANNELS (NUM_SERVOS + 1)
#define NUM_CAPTURES 10
#define NUM_HISTORY 2

class Sensors
{
	public:
		static void init();

		static void startConversion();

		static uint16_t getServoADC(uint8_t index);
		static uint16_t getTouchSensor();
};

#endif /* SENSORS_H_ */
