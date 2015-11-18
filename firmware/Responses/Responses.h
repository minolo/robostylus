#ifndef RESPONSES_H_
#define RESPONSES_H_

#include <stdint.h>
#include <Servo/Servo.h>

#define HEAD_BYTE '['
#define TAIL_BYTE ']'

#define RESPONSE_SIZE sizeof(struct response_t)

union payload_t
{
	struct
	{
		uint16_t servo_position[NUM_SERVOS];
		uint16_t servo_adc[NUM_SERVOS];
		uint16_t touch;
	};

	uint16_t ticks;
};

struct response_t
{
	uint8_t head_byte;
	uint8_t id;

	union
	{
		union payload_t payload;
		uint8_t payload_raw[sizeof(payload_t)];
	};

	uint8_t checksum;
	uint8_t tail_byte;
};

enum RESPONSE_TYPE
{
	RES_ERR_BAD_CHECKSUM 		= 0x80,
	RES_ERR_INVALID_CMD 		= 0x81,

	RES_RESET					= 0x90,
	RES_SENSORS					= 0x91,
	RES_ANIMATIONFINISHED		= 0x92
};

class Responses
{
	public:
		static void init();

		static void setServoPosition(uint8_t index, uint16_t position);
		static void setServoADC(uint8_t index, uint16_t adc);
		static void setTouch(uint16_t touch);

		static void setTicks(uint16_t ticks);

		static void queue(uint8_t type);
};

#endif /* RESPONSES_H_ */
