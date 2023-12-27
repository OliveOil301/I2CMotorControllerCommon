#ifndef I2CMotorControllerCommon_h
#define I2CMotorControllerCommon_h
#include <Arduino.h>

//This is the drive type for the motor driver. 
//If coasting is not needed, MODE_DRIVE_BRAKE should 
//be used as it is computationally easier but there is little actual difference
enum DriveType
{
	MODE_DRIVE_BRAKE, //Phase/Enable mode
	MODE_DRIVE_BRAKE_COAST //PWM or IN/IN mode
};



/** @name CommandType
 *
 * @brief this typedef is used during I2C communication to define exactly what the raw data that is transmitted is supposed to represent
 */
enum CommandType : uint8_t
{
	//Set
	CMD_SET_DRIVE_TYPE,
	CMD_SET_POSITION,
	CMD_SET_SPEED,
	CMD_SET_COAST,

	//Get
	CMD_GET_POSITION,
	CMD_GET_SPEED,
	CMD_GET_CURRENT,
	CMD_GET_SPEED_GOAL,
	CMD_GET_POSITION_GOAL,

	//Info
	CMD_AT_SPEED,
	CMD_AT_POSITION
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