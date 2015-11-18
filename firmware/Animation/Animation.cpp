#include <Animation/Animation.h>
#include <Servo/Servo.h>
#include <Events/Events.h>
#include <Sensors/Sensors.h>

// Coefficients for the animation polynomial for each servo
static struct
{
	int32_t coeffX2, coeffX1, coeffX0;
} animation_coeffs[NUM_SERVOS];

// Variables to control animation
static bool moving;
static uint16_t tick_count;
static int32_t current_tick, current_tick_2;
static uint16_t sensor_limit;

void Animation::init()
{
	sensor_limit = 0x3FF;
}

// Function to generate the next step in the animation
bool Animation::tick()
{
	if(moving)
	{
		// Check that the movement is safe and still not finished
		if(Sensors::getTouchSensor() < sensor_limit && current_tick < tick_count)
		{
			// Update each servo
			for(uint8_t i = 0; i < NUM_SERVOS; i++)
			{
				uint16_t position = (uint16_t)(((animation_coeffs[i].coeffX2 * current_tick_2 + animation_coeffs[i].coeffX1 * current_tick) >> ANIMATION_COEFFS_POWER) + animation_coeffs[i].coeffX0);
				Servo::setPosition(i, position);
			}

			// Get ready for next tick
			current_tick++;
			current_tick_2 += (current_tick << 1) - 1;
		}
		else
		{
			// Finish movement
			moving = false;
			return true;
		}
	}

	return false;
}

void Animation::setTicks(uint16_t ticks)
{
	tick_count = ticks;
}

uint16_t Animation::getTickCounter()
{
	return (uint16_t)current_tick;
}

void Animation::start()
{
	current_tick = 0;
	current_tick_2 = 0;
	moving = true;
}

void Animation::setSensorLimit(uint16_t limit)
{
	sensor_limit = limit;
}

void Animation::setCoeffX0(uint16_t index, int32_t coeff)
{
	animation_coeffs[index].coeffX0 = coeff;
}

void Animation::setCoeffX1(uint16_t index, int32_t coeff)
{
	animation_coeffs[index].coeffX1 = coeff;
}

void Animation::setCoeffX2(uint16_t index, int32_t coeff)
{
	animation_coeffs[index].coeffX2 = coeff;
}
