#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <stdint.h>

#define HEAD_BYTE '['
#define TAIL_BYTE ']'

#define COMMAND_SIZE sizeof(struct command_t)

struct command_t
{
	uint8_t id;
	uint8_t index;
	int32_t value;
	uint8_t checksum;
};

enum COMMAND_TYPE
{
	CMD_SERVO_SETLOWERLIMIT 	= 0x00,
	CMD_SERVO_SETUPPERLIMIT 	= 0x01,

	CMD_ANIM_SETCOEFFX0			= 0x10,
	CMD_ANIM_SETCOEFFX1			= 0x11,
	CMD_ANIM_SETCOEFFX2			= 0x12,
	CMD_ANIM_SETPRESSLIMIT		= 0x13,
	CMD_ANIM_SETTICKS			= 0x14,
	CMD_ANIM_START				= 0x15
};

class Commands
{
	public:
		static void init();

		static bool receive();
		static void process();
};

#endif /* COMMANDS_H_ */
