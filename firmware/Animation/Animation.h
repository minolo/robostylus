#ifndef ANIMATION_H_
#define ANIMATION_H_

#include <stdint.h>

// Power of 2 by which the animation coefficients are multiplied
#define ANIMATION_COEFFS_POWER 16

class Animation
{
	public:
		static void init();

		static bool tick();

		static void setTicks(uint16_t ticks);
		static uint16_t getTickCounter();

		static void start();

		static void setSensorLimit(uint16_t limit);
		static void setCoeffX0(uint16_t index, int32_t coeff);
		static void setCoeffX1(uint16_t index, int32_t coeff);
		static void setCoeffX2(uint16_t index, int32_t coeff);
};

#endif /* ANIMATION_H_ */
