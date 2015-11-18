#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include <stdint.h>

// The size of the buffer must be a power of 2.
// The binary representation of the size must have a bit size of at least one bit less than the _head and _tail variables.
// If it needs to be higher, the type of the _size* variables and the _head and _tail must be changed to uint16_t or uint32_t accordingly.
// For this implementation, using uint8_t for the variables, the maximum size is 128.
class CircularBuffer
{
    public:
        CircularBuffer(uint8_t *contents, uint8_t size);

		void        write(uint8_t data);
		uint8_t     read();

        uint8_t     used();
		uint8_t     free();

	private:
		volatile uint8_t    *_contents;
        volatile uint8_t    _size, _size_mask;
		volatile uint8_t    _head, _tail;
};

#endif /* CIRCULARBUFFER_H_ */
