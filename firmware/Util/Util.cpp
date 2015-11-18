#include <Util/Util.h>

uint16_t Util::clamp(uint16_t value, uint16_t lower_limit, uint16_t upper_limit)
{
	if(value > upper_limit) return upper_limit;
	if(value < lower_limit) return lower_limit;
	return value;
}
