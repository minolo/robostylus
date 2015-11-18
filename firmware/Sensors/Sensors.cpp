#include <Sensors/Sensors.h>
#include <Util/Util.h>
#include <Events/Events.h>
#include <Servo/Servo.h>

#include <avr/interrupt.h>

static volatile uint16_t adc_values[NUM_HISTORY][NUM_CHANNELS];
static volatile uint16_t adc_values_avg[NUM_CHANNELS];
static volatile uint8_t current_capture_history;

////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINES
////////////////////////////////////////////////////////////////////////////////////////
ISR(ADC_vect)
{
	static uint8_t current_channel;
	static uint8_t current_capture;

	// Read ADC value for current channel
	uint16_t current_adc = ADCW;

	// Store minimum value among all captures for the channel
	if(current_adc < adc_values[current_capture_history][current_channel])
	{
		adc_values[current_capture_history][current_channel] = current_adc;
	}

	// Next channel
	next_mod_n(current_channel, NUM_CHANNELS);
	ADMUX = (ADMUX & 0xF8) | (current_channel & 0x07);

	// Continue ADC operation or notify if finished.
	next_mod_n(current_capture, NUM_CHANNELS * NUM_CAPTURES);
	if(current_capture != 0)
	{
		ADCSRA |= _BV(ADSC);
	}
	else
	{
		next_mod_n(current_capture_history, NUM_HISTORY);

		// Calculate moving average for each channel
		for(uint8_t channel = 0; channel < NUM_CHANNELS; channel++)
		{
			uint32_t avg = 0;
			for(uint8_t capture = 0; capture < NUM_HISTORY; capture++)
			{
				avg += adc_values[capture][channel];
			}

			adc_values_avg[channel] = avg / NUM_HISTORY;
		}

		// Notify that the conversion is finished
		Events::queueEvent(EVENT_ADC);
	}
}

////////////////////////////////////////////////////////////////////////////////////////
// CLASS FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////
void Sensors::init()
{
	// Select AREF reference voltage
	ADMUX &= ~(_BV(REFS1) | _BV(REFS0));

	// Right adjust result
	ADMUX &= ~(_BV(ADLAR));

	// Enable ADC interrupt
	ADCSRA |= _BV(ADIE);

	// Select 1:128 ADC prescaler
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

	// Select channel 0
	ADMUX &= 0xF0;

	// Set PC0-PC4 as inputs
	DDRC &= ~(_BV(DDC5) - 1);

	// Disable PC0-PC4 pullups
	PORTC &= ~(_BV(PC5) - 1);

	// Disable digital input buffers 0-4
	DIDR0 |= _BV(ADC5D) - 1;

	// Enable ADC
	ADCSRA |= _BV(ADEN);
}

void Sensors::startConversion()
{
	// Reset values
	for(uint8_t i = 0; i < NUM_CHANNELS; i++)
	{
		adc_values[current_capture_history][i] = 0x3FF;
	}

	// Start the ADC
	ADCSRA |= _BV(ADSC);
}

uint16_t Sensors::getServoADC(uint8_t index)
{
	return adc_values_avg[index];
}

uint16_t Sensors::getTouchSensor()
{
	return adc_values_avg[NUM_CHANNELS - 1];
}
