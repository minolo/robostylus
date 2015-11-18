#define BAUD 115200
#include <util/setbaud.h>

#include <Serial/Serial.h>
#include <CircularBuffer/CircularBuffer.h>

#include <avr/interrupt.h>

// Allocate buffers
static uint8_t txBuffer[SERIAL_TXBUFFER_LENGTH];
static uint8_t rxBuffer[SERIAL_RXBUFFER_LENGTH];

// Create circular buffer control structures
static CircularBuffer serial_txBuffer(txBuffer, SERIAL_TXBUFFER_LENGTH);
static CircularBuffer serial_rxBuffer(rxBuffer, SERIAL_RXBUFFER_LENGTH);

////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINES
////////////////////////////////////////////////////////////////////////////////////////
// Interrupt generated when the serial TX buffer is free
ISR(USART_UDRE_vect)
{
	if(serial_txBuffer.used() > 0)
	{
		UDR0 = serial_txBuffer.read();
	}
	else
	{
		// Disable interrupt
		UCSR0B &= ~_BV(UDRIE0);
	}
}

// Interrupt generated when a byte has been received via serial
ISR(USART_RX_vect)
{
	uint8_t received_byte = UDR0;

	if(serial_rxBuffer.free() > 0)
	{
		serial_rxBuffer.write(received_byte);
	}
}

////////////////////////////////////////////////////////////////////////////////////////
// CLASS FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////
void Serial::init()
{
	// Disable USART module to change configuration
	UCSR0B = 0;

	// Set baud rate
	UBRR0 = UBRR_VALUE;

	// Use double speed if needed
	#if USE_2X
		UCSR0A |= _BV(U2X0);
	#else
		UCSR0A &= ~_BV(U2X0);
	#endif

	// Enable USART receive and transmit
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);

	// Enable USART receive interrupt
	UCSR0B |= _BV(RXCIE0);

	// Set 8-bit character size
	UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);
}

bool Serial::write(uint8_t *buffer, uint8_t length)
{
	if(length > serial_txBuffer.free())
	{
		return false;
	}

	for(uint8_t i = 0; i < length; i++)
	{
		serial_txBuffer.write(buffer[i]);
	}

	// Enable empty transmit buffer interrupt
	UCSR0B |= _BV(UDRE0);

	return true;
}

bool Serial::read(uint8_t *buffer, uint8_t length)
{
	if(length > serial_rxBuffer.used())
	{
		return false;
	}

	for(uint8_t i = 0; i < length; i++)
	{
		buffer[i] = serial_rxBuffer.read();
	}

	return true;
}

void Serial::blockingWrite(uint8_t *buffer, uint8_t length)
{
	while(!Serial::write(buffer, length));
}
