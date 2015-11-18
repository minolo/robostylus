#ifndef EVENTS_H_
#define EVENTS_H_

#include <stdint.h>

#define EVENT_BUFFER_LENGTH 8

enum EVENT_TYPE
{
	EVENT_TIMER,
	EVENT_ADC
};

class Events
{
	public:
		static void init();

		static void queueEvent(uint8_t event);

		static bool inQueue();
		static void process();
};

#endif /* EVENTS_H_ */
