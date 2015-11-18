#ifndef RSSERIALCONTROLLER_H_
#define RSSERIALCONTROLLER_H_

#include <ros/ros.h>

using namespace std;
using namespace ros;

#define NUM_SERVOS 4
#define COEFF_FACTOR (1 << 16)

enum COMMAND_TYPE
{
	CMD_SERVO_SETLOWERLIMIT 	= 0x00,
	CMD_SERVO_SETUPPERLIMIT 	= 0x01,

	CMD_ANIM_SETCOEFFX0		= 0x10,
	CMD_ANIM_SETCOEFFX1		= 0x11,
	CMD_ANIM_SETCOEFFX2		= 0x12,
	CMD_ANIM_SETPRESSLIMIT		= 0x13,
	CMD_ANIM_SETTICKS		= 0x14,
	CMD_ANIM_START			= 0x15
};

enum RESPONSE_TYPE
{
	RES_ERR_BAD_CHECKSUM 		= 0x80,
	RES_ERR_INVALID_CMD 		= 0x81,

	RES_RESET			= 0x90,
	RES_SENSORS			= 0x91,
	RES_ANIMATIONFINISHED		= 0x92
};

struct packet_tx_t
{
	unsigned char	head;
	unsigned char	command;
	unsigned char	index;
	int 			value;
	unsigned char	checksum;
	unsigned char	tail;
} __attribute__ ((packed));

union payload_t
{
	struct
	{
		unsigned short servo_position[NUM_SERVOS];
		unsigned short servo_adc[NUM_SERVOS];
		unsigned short touch;
	};

	unsigned short ticks;
};

struct packet_rx_t
{
	unsigned char id;

	union
	{
		union payload_t payload;
		unsigned char payload_raw[sizeof(payload_t)];
	};

	unsigned char checksum;
} __attribute__ ((packed));

class RSSerialController
{
	public:
		static void init(NodeHandle& nh);
		static void setSensorPacketCallback(void (*spc)(vector<float>& joints_pos, vector<float>& joints_adc, float pressure));
		static void setMovementFinishedPacketCallback(void (*mfpc)(unsigned short ticks_taken));

		static bool connected();

		static void setServoLimits(int index, float lower, float upper);

		static void setCoeffs(int index, float coeffX2, float coeffX1, float coeffX0);
		static void setPressureLimit(float limit);
		static void setTickCount(unsigned short ticks);
		static void start();

		static void run();
};

#endif /* RSSERIALCONTROLLER_H_ */
