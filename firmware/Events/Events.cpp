#include <Events/Events.h>
#include <CircularBuffer/CircularBuffer.h>
#include <Sensors/Sensors.h>
#include <Servo/Servo.h>
#include <Commands/Commands.h>
#include <Responses/Responses.h>
#include <Animation/Animation.h>

static uint8_t event_buffer[EVENT_BUFFER_LENGTH];
static CircularBuffer event_queue(event_buffer, EVENT_BUFFER_LENGTH);

void Events::init()
{

}

void Events::queueEvent(uint8_t event)
{
	event_queue.write(event);
}

bool Events::inQueue()
{
	return event_queue.used() > 0;
}

void Events::process()
{
	uint8_t current_event = event_queue.read();

	switch(current_event)
	{
		case EVENT_TIMER:
			// Start ADC sampling
			Sensors::startConversion();

			// Tick servo relay
			Servo::relayTick();
			break;

		case EVENT_ADC:
			// Set servo ADC
			for(uint8_t i = 0; i < NUM_SERVOS; i++)
			{
				Servo::setADC(i, Sensors::getServoADC(i));
			}

			// Prepare sensors message payload
			for(uint8_t i = 0; i < NUM_SERVOS; i++)
			{
				Responses::setServoPosition(i, Servo::getPosition(i));
				Responses::setServoADC(i, Servo::getADCPosition(i));
			}
			Responses::setTouch(Sensors::getTouchSensor());

			// Send sensors message
			Responses::queue(RES_SENSORS);

			// Tick the animation and check if it finished
			if(Animation::tick())
			{
				Responses::setTicks(Animation::getTickCounter());
				Responses::queue(RES_ANIMATIONFINISHED);
			}

			break;

		default:
			break;
	}
}
