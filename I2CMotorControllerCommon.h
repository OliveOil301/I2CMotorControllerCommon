#ifndef I2CMotorControllerCommon_h
#define I2CMotorControllerCommon_h
#include <Arduino.h>

enum CommunicationMethod : uint8_t
{
	COMM_I2C,
	COMM_UART,
	COMM_SPI
};

enum GoalType : uint8_t
{
	GOAL_UNKNOWN = 0,
	GOAL_POSITION_GO = 1,
	GOAL_POSITION_HOLD = 2,
	GOAL_VELOCITY_GO = 3,
	GOAL_VELOCITY_HOLD = 4
};

enum LimitSwitch : uint8_t
{
	LIMIT_SWITCH_HOME = 1,
	LIMIT_SWITCH_END = 2,
};


enum HomingMethod : uint8_t
{
	HOMING_NONE = 0,
	HOMING_LIMIT_SWITCH_NEG = 1,
	HOMING_LIMIT_SWITCH_POS = 2,
	HOMING_POSITION_ZERO = 3,
	HOMING_POSITION_VALUE = 4,
};

enum PositionControlMethod : uint8_t
{
	POSITION_NONE = 0,
	POSITION_POT = 1,
	POSITION_ENCODER = 2,
	POSITION_TIME = 3
};

enum ValueType : uint8_t
{
	VALUE_POS_LIM_SWITCH_ENABLE = 1,
	VALUE_NEG_LIM_SWITCH_ENABLE = 2,
	VALUE_POSITION_CONTROL_METHOD = 5,
	VALUE_HOMING_METHOD = 7,
	VALUE_ENDING_METHOD = 8,
	VALUE_POSITION_PID_P = 10,
	VALUE_POSITION_PID_I = 11,
	VALUE_POSITION_PID_D = 12,
	VALUE_VELOCITY_PID_P = 13,
	VALUE_VELOCITY_PID_I = 14,
	VALUE_VELOCITY_PID_D = 15,
	VALUE_ENCODER_TICS = 20,
	VALUE_HOME_POSITION = 25,
	VALUE_END_POSITION = 26,
	VALUE_CURRENT_POSITION = 30,
	VALUE_CURRENT_VELOCITY = 31,
	VALUE_CURRENT_CURRENT = 32,
	VALUE_POSITION_GOAL = 35,
	VALUE_VELOCITY_GOAL = 36,
	//Driver pins
	VALUE_PIN_DRIVER_1 = 37,
	VALUE_PIN_DRIVER_2 = 38,
	VALUE_PIN_DRIVER_3 = 39,
	VALUE_PIN_DRIVER_4 = 40,
	VALUE_PIN_DRIVER_5 = 41,
	VALUE_PIN_DRIVER_6 = 42,
	VALUE_PIN_DRIVER_7 = 43,
	VALUE_PIN_DRIVER_8 = 44,
	VALUE_PIN_DRIVER_9 = 45,
	VALUE_PIN_DRIVER_10 = 46,
	VALUE_PIN_DRIVER_11 = 47,
	VALUE_PIN_DRIVER_12 = 48,
	//position input pins
	VALUE_PIN_INPUT_1 = 61,
	VALUE_PIN_INPUT_2 = 62,
	VALUE_PIN_INPUT_3 = 63,
	VALUE_PIN_INPUT_4 = 64,
	VALUE_PIN_INPUT_5 = 65,
	VALUE_PIN_INPUT_6 = 66,
	VALUE_PIN_INPUT_7 = 67,
	VALUE_PIN_INPUT_8 = 68,
	//Other driver configuration values
	VALUE_DRIVER_CONFIGURATION_1 = 81,
	VALUE_DRIVER_CONFIGURATION_2 = 82,
	VALUE_DRIVER_CONFIGURATION_3 = 83,
	VALUE_DRIVER_CONFIGURATION_4 = 84,
	VALUE_DRIVER_CONFIGURATION_5 = 85,
	VALUE_DRIVER_CONFIGURATION_6 = 86,
	VALUE_DRIVER_CONFIGURATION_7 = 87,
	VALUE_DRIVER_CONFIGURATION_8 = 88,
	VALUE_DRIVER_CONFIGURATION_9 = 89,
	VALUE_DRIVER_CONFIGURATION_10 = 90,
	VALUE_DRIVER_CONFIGURATION_11 = 91,
	VALUE_DRIVER_CONFIGURATION_12 = 92,
	VALUE_DRIVER_CONFIGURATION_13 = 93,
	VALUE_DRIVER_CONFIGURATION_14 = 94,
	VALUE_DRIVER_CONFIGURATION_15 = 95,
	VALUE_DRIVER_CONFIGURATION_16 = 96,


	VALUE_ERROR0 = 128,
	VALUE_ERROR1 = 129,
	VALUE_ERROR2 = 130,
	VALUE_ERROR3 = 131,
};

/** @name CommandType
 *
 * @brief this typedef is used during I2C communication to define exactly what the raw data that is transmitted is supposed to represent
 */
enum CommandType : uint8_t
{
	CMD_UNKNOWN = 0,
	//Init
	CMD_BEGIN = 1,
	CMD_GO_HOME = 25,			//Go to a specified limit switch (oftentimes home)
	CMD_GO_END = 26,
	
	//Set
	CMD_SET_GOAL = 30,
	CMD_SET_COAST = 35, //Just coast
	CMD_SET_BRAKE = 36, //Brake the motor
	CMD_MAINTAIN_POSITION = 37, // Maintain the current position actively using the set PID values

	//Get/set configuration and other values
	CMD_GET = 40, 
	CMD_SET = 41,
};


struct MotorControllerMessage
{
	MotorControllerMessage()
	{
		command = CMD_UNKNOWN;
    	key = 0;
    	value = 0;
		for(int i = 0; i<8; i++)
		{
    		data[i] = 0;
		}

	}
    MotorControllerMessage(uint8_t dataIn[8])
    {
        //Save the command
        command = (CommandType)dataIn[0];
        //Save the key
        key = (((uint16_t)dataIn[2])<<8) | (((uint16_t)dataIn[1]));
        //Save the value
        value = (((uint32_t)dataIn[6])<<24) | (((uint32_t)dataIn[5])<<16) | (((uint32_t)dataIn[4])<<8) | (((uint32_t)dataIn[3]));
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
        //Copy the data in case it got changed
        data[0] = command;
        data[1] = (key&0x00FF);
        data[2] = (key&0xFF00)>>8;
        data[3] = (value&0x000000FF);
        data[4] = (value&0x0000FF00)>>8;
        data[5] = (value&0x00FF0000)>>16;
        data[6] = (value&0xFF000000)>>24;
        data[7] = 0xFF;
    }

    void toData(uint8_t *&dataOut)
    {
        //Copy the data in case it got changed
        data[0] = command;
        data[1] = (key&0x00FF);
        data[2] = (key&0xFF00)>>8;
        data[3] = (value&0x000000FF);
        data[4] = (value&0x0000FF00)>>8;
        data[5] = (value&0x00FF0000)>>16;
        data[6] = (value&0xFF000000)>>24;
        data[7] = 0xFF;

        //Copy our data to dataOut
        memcpy(dataOut, data, 8);
    }

	const String toString(bool hex) const
    {
		if(!hex)
		{
			return "Command: " + String(command) + " Key: " + String(key) + " Value: " + String(value);
		}
		else
		{
			return "0x" + String(data[0], 16) + " 0x" + String(data[1], 16) +  + " 0x" + String(data[2], 16) +  + " 0x" + String(data[3], 16) +  + " 0x" + String(data[4], 16) +  + " 0x" + String(data[5], 16) +  + " 0x" + String(data[6], 16) +  + " 0x" + String(data[7], 16);
		}
    }

    CommandType command;
    uint16_t key;
    uint32_t value;
    uint8_t data[8];
};



inline uint32_t floatToUInt(float f) {
	uint32_t intval;
	memcpy(&intval, &f, sizeof(intval));
	//uint32_t intval = *reinterpret_cast<uint32_t*>(&f);

    return intval;
}

inline float uIntToFloat(uint32_t uintIn) {
	float fltval;
	memcpy(&fltval, &uintIn, sizeof(fltval));
	//float fltval = *reinterpret_cast<float*>(&uintIn);

	//Return the float value
    return fltval;
}

inline int32_t unsignedIntToSignedInt(uint32_t uintIn) 
{ 
	return (uintIn < 0xFFFFFFFF/2) ? (((int32_t)uintIn)-(0xFFFFFFFF/2)) : ((int32_t)(uintIn-(0xFFFFFFFF/2)));
}

inline uint32_t signedIntToUnsignedInt(int32_t intIn)
{
	return (intIn < 0) ? ((uint32_t)(intIn + (0xFFFFFFFF/2))) : (((uint32_t)intIn) + (0xFFFFFFFF/2));
}


#endif