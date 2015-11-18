#include <Commands/Commands.h>
#include <Serial/Serial.h>
#include <Servo/Servo.h>
#include <Responses/Responses.h>
#include <Events/Events.h>
#include <Animation/Animation.h>

static struct command_t command;
static uint8_t *command_raw  = (uint8_t *)&command;

void Commands::init()
{

}

// Get a command from the serial buffer, one byte at a time
bool Commands::receive()
{
	static uint8_t index = 0;
	static uint8_t current_byte;

	if(Serial::read(&current_byte, 1))
	{
		if(index == 0)
		{
			// Ignore all bytes until the head byte is received
			if(current_byte == HEAD_BYTE)
			{
				index++;
			}
		}
		else if(index < (COMMAND_SIZE + 1))
		{
			command_raw[index - 1] = current_byte;
			index++;
		}
		else
		{
			index = 0;

			return current_byte == TAIL_BYTE;
		}
	}

	return false;
}

// Process the received command
void Commands::process()
{
	// Check the validity of the command
	uint8_t checksum = 0;
	for(uint8_t i = 0; i < COMMAND_SIZE; i++)
	{
		checksum ^= command_raw[i];
	}

	// Send a message if the checksum is not correct
	if(checksum != 0)
	{
		Responses::queue(RES_ERR_BAD_CHECKSUM);
	}
	else
	{
		// Execute the command
		switch(command.id)
		{
			case CMD_SERVO_SETLOWERLIMIT:
				Servo::setLowerLimit(command.index, command.value & 0xFFFF);
				break;

			case CMD_SERVO_SETUPPERLIMIT:
				Servo::setUpperLimit(command.index, command.value & 0xFFFF);
				break;

			case CMD_ANIM_SETCOEFFX0:
				Animation::setCoeffX0(command.index, command.value);
				break;

			case CMD_ANIM_SETCOEFFX1:
				Animation::setCoeffX1(command.index, command.value);
				break;

			case CMD_ANIM_SETCOEFFX2:
				Animation::setCoeffX2(command.index, command.value);
				break;

			case CMD_ANIM_SETPRESSLIMIT:
				Animation::setSensorLimit(command.value & 0xFFFF);
				break;

			case CMD_ANIM_SETTICKS:
				Animation::setTicks(command.value & 0xFFFF);
				break;

			case CMD_ANIM_START:
				Animation::start();
				break;

			default:
				Responses::queue(RES_ERR_INVALID_CMD);
				break;
		}
	}
}
