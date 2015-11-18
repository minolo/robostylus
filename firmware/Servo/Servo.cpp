#include <Servo/Servo.h>
#include <Util/Util.h>
#include <Serial/Serial.h>

#include <avr/io.h>
#include <avr/interrupt.h>

// Coefficients to convert ADC value into approximate timer tick count
uint32_t coeffs_a[NUM_SERVOS] = {36179, 36150, 36302, 36904};
uint32_t coeffs_b[NUM_SERVOS] = {11039841, 11078796, 11071114, 11114900};

// Servo data structure
static volatile struct servo_data_t
{
	uint16_t lower_limit;
	uint16_t upper_limit;

	uint16_t position;

	uint16_t adc_position;
} servo_data[NUM_SERVOS];

uint8_t relay_tick_count;

////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINES
////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER1_OVF_vect)
{
	static uint8_t current_servo;

	PORTB |= _BV(current_servo);
	OCR1A = servo_data[current_servo].position;

	next_mod_n(current_servo, NUM_SERVOS);
}

ISR(TIMER1_COMPA_vect)
{
	PORTB &= ~(_BV(NUM_SERVOS) - 1);
}

////////////////////////////////////////////////////////////////////////////////////////
// CLASS FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////
void Servo::init()
{
	///////////////////////////////////
	// CONFIG IO
	///////////////////////////////////

	// Set servo pins as outputs
	DDRB |= (_BV(NUM_SERVOS) - 1);

	// Set servo pins to logic level low
	PORTB &= ~(_BV(NUM_SERVOS) - 1);

	// Set relay pin as output
	DDRD |= _BV(DDD7);

	// Set relay pin to logic level low
	PORTD &= ~_BV(PD7);

	///////////////////////////////////
	// CONFIG TIMER1
	///////////////////////////////////

	// Enable Timer1 Match Compare-A interrupt
	TIMSK1 |= _BV(OCIE1A);

	// No Compare Match outputs, normal operation of Timer1
	TCCR1A = 0;

	// Set clock source for Timer1 to prescaled clk 1:1
	TCCR1B = _BV(CS10);

	// Initialize data structure
	for(uint8_t i = 0; i < NUM_SERVOS; i++)
	{
		servo_data[i].lower_limit = 0x0000;
		servo_data[i].upper_limit = 0xFFFF;
	}

	// Initialize the relay tick count
	relay_tick_count = 0xFF;
}

void Servo::relayTick()
{
	if(relay_tick_count != 0xFF)
	{
		relay_tick_count++;
	}

	if(relay_tick_count < RELAY_ON_TIME)
	{
		// Disable relay
		PORTD &= ~_BV(PD7);

		// Enable Timer1 Overflow interrupt
		TIMSK1 |= _BV(TOIE1);
	}
	else if(relay_tick_count > RELAY_OFF_TIME)
	{
		// Disable relay
		PORTD &= ~_BV(PD7);
	}
	else
	{
		// Enable relay
		PORTD |= _BV(PD7);

		// Disable Timer1 Overflow interrupt
		TIMSK1 &= ~_BV(TOIE1);
	}
}

void Servo::setUpperLimit(uint8_t index, uint16_t upper_limit)
{
	if(upper_limit > servo_data[index].lower_limit)
	{
		servo_data[index].upper_limit = upper_limit;
	}
}

void Servo::setLowerLimit(uint8_t index, uint16_t lower_limit)
{
	if(lower_limit < servo_data[index].upper_limit)
	{
		servo_data[index].lower_limit = lower_limit;
	}
}

void Servo::setPosition(uint8_t index, uint16_t position)
{
	if((servo_data[index].lower_limit > 0x0000) && (servo_data[index].upper_limit < 0xFFFF))
	{
		servo_data[index].position = Util::clamp(position, servo_data[index].lower_limit, servo_data[index].upper_limit);
		relay_tick_count = 0;
	}
}

void Servo::setADC(uint8_t index, uint16_t adc_value)
{
	// Update value only if the relay is disabled
	if((relay_tick_count < RELAY_ON_TIME) || relay_tick_count > RELAY_OFF_TIME + 5)
	{
		uint32_t position = ((((uint32_t)adc_value) * coeffs_a[index]) + coeffs_b[index]) >> SERVO_COEFFS_POWER;
		servo_data[index].adc_position = (uint16_t)(position & 0xFFFF);
	}
}

uint16_t Servo::getPosition(uint8_t index)
{
	return (relay_tick_count < RELAY_OFF_TIME + 5) ? servo_data[index].position : servo_data[index].adc_position;
}

uint16_t Servo::getADCPosition(uint8_t index)
{
	return servo_data[index].adc_position;
}
