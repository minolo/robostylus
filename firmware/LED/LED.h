#ifndef LED_H_
#define LED_H_

#include <stdint.h>

class LED
{
	public:
		static void init();

		static void set(uint8_t state);
};

#endif /* LED_H_ */
