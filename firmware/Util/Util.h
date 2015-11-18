#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>

#define next_mod_n(num, mod) if(++num == mod) num = 0

class Util
{
	public:
		static uint16_t clamp(uint16_t value, uint16_t lower_limit, uint16_t upper_limit);
};

#endif /* UTIL_H_ */
