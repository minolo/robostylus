#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdint.h>

#define SERIAL_TXBUFFER_LENGTH 64
#define SERIAL_RXBUFFER_LENGTH 64

class Serial
{
	public:
		static void init();

		static bool write(uint8_t *buffer, uint8_t length);
		static bool read(uint8_t *buffer, uint8_t length);

		static void blockingWrite(uint8_t *buffer, uint8_t length);
};

#endif /* SERIAL_H_ */
