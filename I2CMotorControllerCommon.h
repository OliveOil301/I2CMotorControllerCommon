#ifndef I2CMotorControllerCommon_h
#define I2CMotorControllerCommon_h
#include <Arduino.h>

enum CommunicationMethod
{
	COMM_I2C,
	COMM_UART,
	COMM_SPI
};


enum HomingMethod
{
	HOMING_NONE,
	HOMING_LIMIT_SWITCH_NEG,
	HOMING_LIMIT_SWITCH_POS,
	HOMING_POT_ZERO,
	HOMING_POT_VALUE,
	HOMING_ENCODER_ZERO,
	HOMING_ENCODER_VALUE
};

enum PositionControlMethod
{
	POSITION_NONE,
	POSITION_POT,
	POSITION_ENCODER,
	POSITION_TIME
};

/** @name CommandType
 *
 * @brief this typedef is used during I2C communication to define exactly what the raw data that is transmitted is supposed to represent
 */
enum CommandType : uint8_t
{
	//Init
	CMD_BEGIN,

	CMD_ENABLE_LIM_SWITCH, //Enable a limit switch
	CMD_SET_POSITION_CONTROL_METHOD,
	CMD_SET_HOMING_METHOD, //


	CMD_SET_ENCODER_TICS,	//If using a quaderature encoder, sets the current encoder tick number to the supplied number
	CMD_SET_PID_VALUE,	//Set a P, I, or D value for position or velocity control
	CMD_GO_LIM_SWITCH,			//Go to a specified limit switch (oftentimes home)

	//Set
	CMD_SET_POSITION_GOAL,
	CMD_SET_VELOCITY_GOAL,
	CMD_SET_LOCATION_GOAL, //One of the limit switches (+/-)
	CMD_SET_COAST, //Just coast
	CMD_SET_BRAKE, //Brake the motor
	CMD_MAINTAIN_POSITION, // Maintain the current position actively using the set PID values

	//Get
	CMD_AT_POSITION, //Returns true if this was previously/currently in position control mode and at the position specified
	CMD_GET_POSITION, //Returns the position (encoder ticks, pot value, time, depending on position control method)
	CMD_AT_VELOCITY, //Returns true if this was previously/currently in velocity control mode and at the velocity specified
	CMD_GET_VELOCITY, //Returns the velocity (encoder ticks/s, pot values/s, %duty cycle, depending on position control method)
	CMD_AT_LOCATION, //Returns true if this was previously/currently in location control mode and at the location specified (+/- lim sw)
	CMD_GET_LOCATION, //Returns the limit switch that the actuator is at, if any

	CMD_GET_CURRENT, //Returns the current of the motor at this point in time

	//Get goals
	CMD_GET_POSITION_GOAL,
	CMD_GET_VELOCITY_GOAL,
	CMD_GET_LOCATION_GOAL,

	//Get errors
	CMD_GET_ERRORS,
	CMD_CLEAR_ERRORS


};


struct MotorControllerMessage
{
    MotorControllerMessage(uint8_t dataIn[8])
    {
        //Save the command
        command = (CommandType)data[0];
        //Save the key
        key = (data[2]<<8) | (data[1]);
        //Save the value
        value = (data[6]<<24) | (data[5]<<16) | (data[4]<<8) | (data[3]);
        //Copy the data
        memcpy(data, dataIn, 8);
    }

    MotorControllerMessage(CommandType commandIn, uint16_t keyIn, uint32_t valueIn)
    {
        //Save the command
        command = commandIn;
        //Save the key
        key = keyIn;
        //Save the value
        value = valueIn;
    }

    void MotorControllerMessage::toData(uint8_t *&dataOut)
    {
        //Copy the data in case it got changed
        data[0] = command;
        data[1] = (key&0x00FF);
        data[2] = (key&0xFF00)>>8;
        data[3] = (command&0x000000FF);
        data[4] = (command&0x0000FF00)>>8;
        data[5] = (command&0x00FF0000)>>16;
        data[6] = (command&0xFF000000)>>24;
        data[7] = 0xFF;

        //Copy our data to dataOut
        memcpy(dataOut, data, 8);
    }

    CommandType command;
    uint16_t key;
    uint32_t value;
    uint8_t data[8];
};



/** @name ResponseType
 *
 * @brief This is a typedef that is used during I2C communication to define what data is in the rest of the message
 * The FW and the library will both use these types to unpack the I2C data transmission
 */
enum ResponseType : uint8_t
{
	RSP_BEGIN,
	RSP_VALUE, // Position, velocity, location, or current value
	RSP_AT_SPEED,
	RSP_AT_POSITION
};


#endif