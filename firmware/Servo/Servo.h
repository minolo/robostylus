#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>

#define NUM_SERVOS			4

#define RELAY_ON_TIME		20
#define RELAY_OFF_TIME		30

// Power of 2 by which the servo polynomial coefficients are multiplied
#define SERVO_COEFFS_POWER 10

class Servo
{
	public:
		static void init();

		static void relayTick();

		static void setUpperLimit(uint8_t index, uint16_t upper_limit);
		static void setLowerLimit(uint8_t index, uint16_t lower_limit);

		static void setPosition(uint8_t index, uint16_t position);
		static void setADC(uint8_t index, uint16_t adc_value);

		static uint16_t getPosition(uint8_t index);
		static uint16_t getADCPosition(uint8_t index);
};

#endif /* SERVO_H_ */
