#include "RSSerialController.h"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <cstdlib>

// Serial port identifiers
int serial_fd;
string serial_device;

// Servo data
vector<int> deg90;
vector<int> zeroes;
unsigned short lower_limits[NUM_SERVOS];
unsigned short upper_limits[NUM_SERVOS];

// RX packet
struct packet_rx_t packet_rx;
unsigned char *packet_rx_raw = (unsigned char *)&packet_rx;

// Callbacks
void (*sensorPacketCallback)(vector<float>& joints_pos, vector<float>& joints_adc, float pressure);
void (*movementFinishedPacketCallback)(unsigned short ticks_taken);

// Cycles <-> Radians conversion functions
float cycles_to_rad(unsigned short cycles, unsigned short zero, short deg90)
{
	return ((float)(cycles - zero)) / deg90 * M_PI_2;
}

unsigned short rad_to_cycles(float rad, unsigned short zero, short deg90)
{
	return (unsigned short)((rad * deg90 / M_PI_2) + zero);
}

// Serial functions
void serial_send_cmd(COMMAND_TYPE command, unsigned char index, int value)
{
	struct packet_tx_t packet_tx;

	packet_tx.head = '[';
	packet_tx.command = command;
	packet_tx.index = index;
	packet_tx.value = value;
	packet_tx.checksum = 0;
	packet_tx.tail = ']';

	for(unsigned char *p = &packet_tx.command; p < &packet_tx.checksum; p++)
	{
		packet_tx.checksum ^= *p;
	}

	int ret = write(serial_fd, &packet_tx, sizeof(packet_tx));

	if(ret <= 0)
	{
		ROS_ERROR_THROTTLE(10, "Could not send data to device");
	}
}

void serial_process_packet()
{
	unsigned char checksum = 0;
	for(int i = 0; i < sizeof(packet_rx); i++)
	{
		checksum ^= packet_rx_raw[i];
	}

	if(checksum == 0)
	{
		switch(packet_rx.id)
		{
			case RES_RESET:
				// Send configuration
				for(unsigned char i = 0; i < NUM_SERVOS; i++)
				{
					serial_send_cmd(CMD_SERVO_SETLOWERLIMIT, i, lower_limits[i]);
					serial_send_cmd(CMD_SERVO_SETUPPERLIMIT, i, upper_limits[i]);
				}

				ROS_INFO("Configuration sent to device");
				break;

			case RES_SENSORS:
				if(sensorPacketCallback != 0)
				{
					vector<float> joints_adc, joints_pos;
					for(int i = 0; i < NUM_SERVOS; i++)
					{
						joints_pos.push_back(cycles_to_rad(packet_rx.payload.servo_position[i], zeroes[i], deg90[i]));
						joints_adc.push_back(cycles_to_rad(packet_rx.payload.servo_adc[i], zeroes[i], deg90[i]));
					}

					float pressure = ((float)packet_rx.payload.touch) / 1023;

					sensorPacketCallback(joints_pos, joints_adc, pressure);
				}
				break;

			case RES_ANIMATIONFINISHED:
				if(movementFinishedPacketCallback != 0)
				{
					movementFinishedPacketCallback(packet_rx.payload.ticks);
				}
				break;

			case RES_ERR_BAD_CHECKSUM:
				ROS_WARN("Received bad checksum response");
				break;

			case RES_ERR_INVALID_CMD:
				ROS_WARN("Received invalid command response");
				break;

			default:
				ROS_WARN("Received bad id");
				break;
		}
	}
	else
	{
		// Dump packet
		stringstream ss;
		for(int i = 0; i < sizeof(packet_rx); i++)
		{
			ss << std::setfill('0') << std::setw(2) << std::hex << std::uppercase << (int)packet_rx_raw[i];
		}

		ROS_WARN("Bad checksum on reception %s", ss.str().c_str());
	}
}

void serial_receive_packet(unsigned char *buffer, int nbytes)
{
	static unsigned int num_byte = 0;

	for(int i = 0; i < nbytes; i++)
	{
		unsigned char current = buffer[i];

		if(num_byte == 0)
		{
			if(current == '[')
			{
				num_byte++;
			}
		}
		else if(num_byte < (sizeof(packet_rx) + 1))
		{
			packet_rx_raw[num_byte - 1] = current;
			num_byte++;
		}
		else
		{
			if(current == ']')
			{
				serial_process_packet();
			}

			num_byte = 0;
		}
	}
}

bool serial_connect()
{
    // Open device file
    serial_fd = open(serial_device.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd == -1)
    {
        return false;
    }

    // Load terminal options
    struct termios options;
    tcgetattr(serial_fd, &options);

    // Set speed to 115200 baud
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // Set raw mode
    cfmakeraw(&options);

    // Enable receiver and ignore modem control lines
    options.c_cflag |= (CLOCAL | CREAD);

    // Set 8N1 mode
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    options.c_cflag |= CS8;

    // Disable flow control
    options.c_cflag &= ~CRTSCTS;

    // Apply changes
    tcsetattr(serial_fd, TCSANOW, &options);

    // Flush all data
    tcflush(serial_fd, TCIOFLUSH);

    return true;
}

bool RSSerialController::connected()
{
	struct termios options;

	return tcgetattr(serial_fd, &options) != -1;
}

void RSSerialController::run()
{
	int nBytes;
	unsigned char buffer[256];

	ROS_INFO("Connecting to device %s", serial_device.c_str());

	while(true)
	{
		while(!serial_connect())
		{
			sleep(1);
		}

		ROS_INFO("Connected to %s", serial_device.c_str());

		while(true)
		{
			nBytes = read(serial_fd, buffer, sizeof(buffer));
			if(nBytes > 0)
			{
				serial_receive_packet(buffer, nBytes);
			}
			else if(nBytes == 0)
			{
				if(!RSSerialController::connected())
				{
					ROS_ERROR("Serial connection lost. Trying to reconnect...");
					break;
				}
			}
		}

		close(serial_fd);
	}
}

void RSSerialController::init(NodeHandle &nh)
{
	////////////////////////////////////////////////////////////
	// Get serial device
	////////////////////////////////////////////////////////////
	if(!nh.getParam("device", serial_device))
	{
		ROS_ERROR("Parameter not set: %s/device", nh.getNamespace().c_str());
		exit(-1);
	}

	////////////////////////////////////////////////////////////
	// Get zeroes parameter and verify it
	////////////////////////////////////////////////////////////
	if(!nh.getParam("zeroes", zeroes))
	{
		ROS_ERROR("Parameter not set: %s/zeroes", nh.getNamespace().c_str());
		exit(-1);
	}

	if(zeroes.size() != NUM_SERVOS)
	{
		ROS_ERROR("%s/zeroes has %d elements, it should have %d", nh.getNamespace().c_str(), (int)zeroes.size(), NUM_SERVOS);
		exit(-1);
	}

	for(vector<int>::iterator it = zeroes.begin(); it != zeroes.end(); it++)
	{
		if(*it < 0)
		{
			ROS_ERROR("%s/zeroes contains negative values", nh.getNamespace().c_str());
			exit(-1);
		}
	}

	////////////////////////////////////////////////////////////
	// Get deg90 parameter and verify it
	////////////////////////////////////////////////////////////
	if(!nh.getParam("deg90", deg90))
	{
		ROS_ERROR("Parameter not set: %s/deg90", nh.getNamespace().c_str());
		exit(-1);
	}

	if(deg90.size() != NUM_SERVOS)
	{
		ROS_ERROR("%s/deg90 has %d elements, it should have %d", nh.getNamespace().c_str(), (int)deg90.size(), NUM_SERVOS);
		exit(-1);
	}
}

void RSSerialController::setSensorPacketCallback(void (*spc)(vector<float>& joints_pos, vector<float>& joints_adc, float pressure))
{
	sensorPacketCallback = spc;
}

void RSSerialController::setMovementFinishedPacketCallback(void (*mfpc)(unsigned short ticks_taken))
{
	movementFinishedPacketCallback = mfpc;
}

void RSSerialController::setServoLimits(int index, float lower, float upper)
{
	unsigned short cycles1 = rad_to_cycles(upper, zeroes.at(index), deg90.at(index));
	unsigned short cycles2 = rad_to_cycles(lower, zeroes.at(index), deg90.at(index));

	upper_limits[index] = std::max<unsigned short>(cycles1, cycles2);
	lower_limits[index] = std::min<unsigned short>(cycles1, cycles2);
}

void RSSerialController::setCoeffs(int index, float coeffX2, float coeffX1, float coeffX0)
{
	int coeffX2_int = round((deg90[index] * coeffX2 / M_PI_2) * COEFF_FACTOR);
	serial_send_cmd(CMD_ANIM_SETCOEFFX2, index, coeffX2_int);

	int coeffX1_int = round((deg90[index] * coeffX1 / M_PI_2) * COEFF_FACTOR);
	serial_send_cmd(CMD_ANIM_SETCOEFFX1, index, coeffX1_int);

	int coeffX0_int = round((deg90[index] * coeffX0 / M_PI_2) + zeroes[index]);
	serial_send_cmd(CMD_ANIM_SETCOEFFX0, index, coeffX0_int);
}

void RSSerialController::setPressureLimit(float limit)
{
	float limit_clamped = std::max<float>(0, std::min<float>(limit, 1));
	serial_send_cmd(CMD_ANIM_SETPRESSLIMIT, 0, (int)(limit_clamped * 1023));
}

void RSSerialController::setTickCount(unsigned short ticks)
{
	serial_send_cmd(CMD_ANIM_SETTICKS, 0, ticks);
}

void RSSerialController::start()
{
	serial_send_cmd(CMD_ANIM_START, 0, 0);
}
