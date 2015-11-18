#include <Responses/Responses.h>
#include <Serial/Serial.h>

#include <string.h>

static struct response_t response;

void Responses::init()
{
	response.head_byte = HEAD_BYTE;
	response.tail_byte = TAIL_BYTE;
}

void Responses::setServoPosition(uint8_t index, uint16_t position)
{
	response.payload.servo_position[index] = position;
}

void Responses::setServoADC(uint8_t index, uint16_t adc)
{
	response.payload.servo_adc[index] = adc;
}

void Responses::setTouch(uint16_t touch)
{
	response.payload.touch = touch;
}

void Responses::setTicks(uint16_t ticks)
{
	response.payload.ticks = ticks;
}

void Responses::queue(uint8_t type)
{
	// Fill response struct and calculate checksum
	response.id = type;
	response.checksum = type;
	for(uint8_t i = 0; i < sizeof(response.payload); i++)
	{
		response.checksum ^= response.payload_raw[i];
	}

	// Write to serial buffers
	Serial::blockingWrite((uint8_t *)&response, RESPONSE_SIZE);

	// Reset payload
	memset(&response.payload, 0, sizeof(response.payload));
}
