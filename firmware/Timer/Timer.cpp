#include <Timer/Timer.h>
#include <Events/Events.h>

#include <avr/interrupt.h>

ISR(TIMER0_OVF_vect)
{
	Events::queueEvent(EVENT_TIMER);
}

void Timer::init()
{
	///////////////////////////////////
	// CONFIG TIMER0
	///////////////////////////////////

	// Enable Timer0 Overflow interrupt
	TIMSK0 |= _BV(TOIE0);

	// Set clock source for Timer0 to prescaled clk 1:1024
	TCCR0B = _BV(CS02) | _BV(CS00);
}
