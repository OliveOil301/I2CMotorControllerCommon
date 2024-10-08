#ifndef I2CMotorControllerCommon_h
#define I2CMotorControllerCommon_h
#include <Arduino.h>

enum CommunicationMethod
{
	COMM_I2C,
	COMM_UART,
	COMM_SPI
};

enum GoalType
{
	GOAL_UNKNOWN = 0,
	GOAL_POSITION_GO = 1,
	GOAL_POSITION_HOLD = 2,
	GOAL_VELOCITY_GO = 3,
	GOAL_VELOCITY_HOLD = 4
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

enum ValueType
{
	VALUE_POS_LIM_SWITCH_ENABLE = 1,
	VALUE_NEG_LIM_SWITCH_ENABLE = 2,
	VALUE_POSITION_CONTROL_METHOD = 5,
	VALUE_HOMING_METHOD = 7,
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
	VALUE_ERROR = 50,
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
	CMD_GO_LIM_SWITCH = 25,			//Go to a specified limit switch (oftentimes home)

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
		Serial.println("Message Command: " + String(command, 10) + " Data: 0x" + String(dataIn[0], 16));
        //Save the key
        key = (((uint16_t)dataIn[2])<<8) | (((uint16_t)dataIn[1]));
		Serial.println("Message Key: " + String(key, 10));
        //Save the value
        value = (((uint32_t)dataIn[6])<<24) | (((uint32_t)dataIn[5])<<16) | (((uint32_t)dataIn[4])<<8) | (((uint32_t)dataIn[3]));
		Serial.println("Message Value: " + String(value, 10));
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

	const String MotorControllerMessage::toString(bool hex) const
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

byte* floatToByteArray(float f) {
	uint32_t intval = *reinterpret_cast<uint32_t*>(&f);
    byte* ret = (byte*)malloc(4 * sizeof(byte));

    int i;
    for (i = 0; i < 4; i++) {
        ret[i] = (intval >> 8 * i) & 0xFF;
    }

    return ret;
}

float byteArrayToFloat(byte* arr) {
	//Initialize a 32-bit uinteger as 0
	uint32_t asInt = 0;
    int i;
	//From 0 to 3
    for (i = 0; i < 4; i++) {
		//OR byte i of the array with the appropriate location in the uinteger we made earlier
		asInt |= ((((uint32_t)arr[i])& 0xFF)<< (8 * i));
		Serial.println("Byte " + String(i) + ": " + String(arr[i], 2) + " Shifted: " + String(asInt, 2));
        //ret[i] = (asInt >> 8 * i) & 0xFF;
    }

	Serial.println("Int representation: 0b" + String(asInt, 2));
	float fltval = *reinterpret_cast<float*>(&asInt);

	//Return the float value
    return fltval;
}


#endif