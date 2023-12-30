#ifndef I2CMotorControllerCommon_h
#define I2CMotorControllerCommon_h
#include <Arduino.h>

enum CommunicationMethod
{
	METHOD_I2C,
	METHOD_SERIAL,
	METHOD_SPI
};

//This is the drive type for the motor driver. 
//If coasting is not needed, MODE_DRIVE_BRAKE should 
//be used as it is computationally easier but there is little actual difference
enum DriveMethod
{
	METHOD_DRIVE_BRAKE, //Phase/Enable mode
	METHOD_DRIVE_BRAKE_COAST //PWM or IN/IN mode
};

enum HomingMethod
{
	METHOD_NEG_LIMIT_SWITCH,
	METHOD_POS_LIMIT_SWITCH,
	METHOD_POT_ZERO,
	METHOD_ENCODER_ZERO
};

/** @name CommandType
 *
 * @brief this typedef is used during I2C communication to define exactly what the raw data that is transmitted is supposed to represent
 */
enum CommandType : uint8_t
{
	//Init
	CMD_INIT,
	CMD_SET_HOMING_METHOD,
	CMD_SET_ENCODER_TICS,
	CMD_SET_POSITION_PID,
	CMD_SET_VELOCITY_PID,
	CMD_FIND_HOME,
	CMD_HOME,

	//Set
	CMD_SET_DRIVE_TYPE,
	CMD_SET_POSITION_GOAL,
	CMD_SET_VELOCITY_GOAL,
	CMD_SET_LOCATION_GOAL,
	CMD_SET_COAST,

	//Get
	CMD_AT_POSITION,
	CMD_GET_POSITION,
	CMD_AT_VELOCITY,
	CMD_GET_VELOCITY,
	CMD_AT_LOCATION,
	CMD_GET_LOCATION,

	CMD_GET_CURRENT,

	//Get goals
	CMD_GET_SPEED_GOAL,
	CMD_GET_POSITION_GOAL,
	CMD_GET_LOCATION_GOAL,

	//Get errors
	CMD_GET_ERRORS

};





/** @name ResponseType
 *
 * @brief This is a typedef that is used during I2C communication to define what data is in the rest of the message
 * The FW and the library will both use these types to unpack the I2C data transmission
 */
enum ResponseType : uint8_t
{
	RSP_INIT_RESULT,
	RSP_DRIVE_TYPE,
	RSP_POSITION,
	RSP_SPEED,
	RSP_CURRENT,
	RSP_AT_SPEED,
	RSP_AT_POSITION
};


#endif