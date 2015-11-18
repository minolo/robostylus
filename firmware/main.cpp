#include <avr/interrupt.h>

#include <Serial/Serial.h>
#include <Sensors/Sensors.h>
#include <Commands/Commands.h>
#include <LED/LED.h>
#include <Servo/Servo.h>
#include <Events/Events.h>
#include <Timer/Timer.h>
#include <Responses/Responses.h>
#include <Animation/Animation.h>

#include <util/delay.h>

int main(void)
{
	// Initialize everything
	cli();
	Animation::init();
	Responses::init();
	Commands::init();
	Sensors::init();
	Events::init();
	Serial::init();
	Timer::init();
	Servo::init();
	LED::init();
	sei();

	// Send a reset packet
	Responses::queue(RES_RESET);

	while(1)
	{
		// Process commands
		if(Commands::receive())
		{
			LED::set(1);
			Commands::process();
		}

		// Process events
		if(Events::inQueue())
		{
			LED::set(1);
			Events::process();
		}

		LED::set(0);
	}

	return 0;
}
