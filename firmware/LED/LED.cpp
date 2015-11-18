#include <LED/LED.h>

#include <avr/io.h>

void LED::init()
{
	// Set PB5 to output
	DDRB |= _BV(PB5);

	// Set PB5 to logic level low
	PORTB &= ~_BV(PB5);
}

void LED::set(uint8_t state)
{
	if(state)
	{
		PORTB |= _BV(PB5);
	}
	else
	{
		PORTB &= ~_BV(PB5);
	}
}
