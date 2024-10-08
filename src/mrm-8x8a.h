#pragma once
#include "Arduino.h"
#include <mrm-board.h>
#include <mrm-common.h>
//#include <vector>

/**
Purpose: mrm-8x8a interface to CANBus.
@author MRMS team
@version 0.3 2019-09-07
Licence: You can use this code any way you like.
*/

#define CAN_ID_8x8A0_IN 0x200
#define CAN_ID_8x8A0_OUT 0x201
#define CAN_ID_8x8A1_IN 0x202
#define CAN_ID_8x8A1_OUT 0x203
#define CAN_ID_8x8A2_IN 0x204
#define CAN_ID_8x8A2_OUT 0x205
#define CAN_ID_8x8A3_IN 0x206
#define CAN_ID_8x8A3_OUT 0x207
#define CAN_ID_8x8A4_IN 0x208
#define CAN_ID_8x8A4_OUT 0x209
#define CAN_ID_8x8A5_IN 0x20A
#define CAN_ID_8x8A5_OUT 0x20B
#define CAN_ID_8x8A6_IN 0x20C
#define CAN_ID_8x8A6_OUT 0x20D
#define CAN_ID_8x8A7_IN 0x20E
#define CAN_ID_8x8A7_OUT 0x20F

//CANBus commands
#define COMMAND_8X8_DISPLAY 0x00
#define COMMAND_8X8_SWITCH_ON 0x01
#define COMMAND_8X8_SWITCH_ON_REQUEST_NOTIFICATION 0x02
#define COMMAND_8x8_TEST_CAN_BUS 0x03
#define COMMAND_8X8_BITMAP_DISPLAY_PART1 0x05
#define COMMAND_8X8_BITMAP_DISPLAY_PART2 0x06
#define COMMAND_8X8_BITMAP_DISPLAY_PART3 0x07
#define COMMAND_8X8_BITMAP_STORE_PART1 0x08
#define COMMAND_8X8_BITMAP_STORE_PART2 0x09
#define COMMAND_8X8_BITMAP_STORE_PART3 0x0A
#define COMMAND_8X8_BITMAP_STORED_DISPLAY 0x0B
#define COMMAND_8X8_ROTATION_SET 0x0C
#define COMMAND_8X8_TEXT_1 0x50
#define COMMAND_8X8_TEXT_2 0x51
#define COMMAND_8X8_TEXT_3 0x52
#define COMMAND_8X8_TEXT_4 0x53
#define COMMAND_8X8_TEXT_5 0x54
#define COMMAND_8X8_TEXT_6 0x55

#define MRM_8x8A_SWITCHES_COUNT 4
#define MRM_8X8A_TEXT_LENGTH 44

#define MRM_8X8A_INACTIVITY_ALLOWED_MS 30000

class ActionBase;

class Mrm_8x8a : public SensorBoard
{
	bool _activeCheckIfStarted = true;
	std::vector<uint8_t>* displayedLast;
	std::vector<uint8_t>* displayedTypeLast;
	std::vector<bool[MRM_8x8A_SWITCHES_COUNT]>* lastOn;
	std::vector<bool[MRM_8x8A_SWITCHES_COUNT]>* on;
	std::vector<ActionBase *[MRM_8x8A_SWITCHES_COUNT]>* offOnAction;

	/** If sensor not started, start it and wait for 1. message
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - started or not
	*/
	bool started(uint8_t deviceNumber);
	
public:
	struct LEDSign{
		uint8_t type;
	};

	struct LEDSignBitmap : LEDSign{
		uint8_t red[8] = {0, 0, 0, 0, 0, 0, 0, 0};
		uint8_t green[8] =  {0, 0, 0, 0, 0, 0, 0, 0};;
		LEDSignBitmap(){type = 0;}
	};

	struct LEDSignText : LEDSign{
		char text[8];
		LEDSignText(){type = 1;}
	};

	enum LED8x8Rotation { LED_8X8_BY_0_DEGREES, LED_8X8_BY_90_DEGREES, LED_8X8_BY_270_DEGREES };

	enum LED8x8Type{LED_8X8_CUSTOM, LED_8X8_STORED, LED_8X8_STORED_CUSTOM };

	enum ledSign {LED_CATCH, LED_CUSTOM, LED_EVACUATION_ZONE, LED_FULL_CROSSING_BOTH_MARKS, LED_FULL_CROSSING_MARK_LEFT,
		LED_FULL_CROSSING_MARK_RIGHT, LED_FULL_CROSSING_NO_MARK, LED_GOAL_APPROACH, LED_HALF_CROSSING_MARK_LEFT, LED_HALF_CROSSING_MARK_RIGHT, 
		LED_HALF_CROSSING_LEFT_NO_MARK, LED_HALF_CROSSING_RIGHT_NO_MARK, LED_IDLE, LED_IMU_FOLLOW, LED_LINE_AVOID, LED_LINE_FULL, 
		LED_LINE_FULL_BOTH_MARKS, LED_LINE_FULL_MARK_LEFT, LED_LINE_FULL_MARK_RIGHT, LED_LINE_INTERRUPTED, LED_CURVE_LEFT,
		LED_CURVE_RIGHT, LED_OBSTACLE, LED_OBSTACLE_AROUND_LEFT, LED_OBSTACLE_AROUND_RIGHT, LED_PAUSE, LED_PLAY, LED_T_CROSSING_BY_L,
		LED_T_CROSSING_BY_R, LED_WALL_AHEAD, LED_WALL_DOWN_FOLLOW, LED_WALL_L, LED_WALL_LEFT_FOLLOW, LED_WALL_R, 
		 LED_WALL_RIGHT_FOLLOW, LED_WALL_UP_FOLLOW }; 
	
	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Mrm_8x8a(Robot* robot = NULL, uint8_t maxDevices = 1);

	~Mrm_8x8a();

	/** Contacting mrm_8x8a periodically to check. Normally true:
	 */
	void activeCheckIfStartedSet(bool value){_activeCheckIfStarted = value;}

	ActionBase* actionCheck();

	void actionSet(ActionBase* action, uint8_t switchNumber, uint8_t deviceNumber = 0);

	/** Add a mrm-8x8a board
	@param deviceName - device's name
	*/
	void add(char * deviceName = (char*)"");

	/** Display stored (in sensor, read-only) bitmap
	@param bitmapId - bitmap's id
	@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void bitmapDisplay(uint8_t bitmapId, uint8_t deviceNumber = 0);

	/** Display custom bitmap
	@param red - 8-byte array for red
	@param green - 8-byte array for green
	@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void bitmapCustomDisplay(uint8_t red[], uint8_t green[], uint8_t deviceNumber = 0);

	/** Store custom bitmap
	@param red - 8-byte array for red
	@param green - 8-byte array for green
	@param addres - address in display's RAM. 0 - 99.
	@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void bitmapCustomStore(uint8_t red[], uint8_t green[], uint8_t address, uint8_t deviceNumber = 0);

	/** Display custom custom bitmap
	@param addres - address in display's RAM. 0 - 99.
	@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void bitmapCustomStoredDisplay(uint8_t address, uint8_t deviceNumber = 0);

	void bitmapsSet(const std::vector<ledSign>& selectedImages);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@param length - number of data bytes
	@return - true if canId for this class
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8], uint8_t dlc = 8);

	/** Displays 8-row progress bar. Useful for visual feedback of a long process.
	@param period - total count (100%)
	@param current - current count (current percentage)
	@param reset - reset to start (no bar)
	@return - display changed
	*/
	bool progressBar(uint32_t period, uint32_t current, bool reset = false);

	/** All readings
	@param subsensorNumberInSensor - like a single IR transistor in mrm-ref-can
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t subsensorNumberInSensor, uint8_t deviceNumber = 0){ return switchRead(subsensorNumberInSensor, deviceNumber);}

	/** Set rotation from now on
	@param rotation - 0, 90, or 270 degrees counterclockwise
	@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void rotationSet(enum LED8x8Rotation rotation = LED_8X8_BY_0_DEGREES, uint8_t deviceNumber = 0);

	/** Read switch
	@param switchNumber - 0 - 3
	@deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - true if pressed, false otherwise
	*/
	bool switchRead(uint8_t switchNumber, uint8_t deviceNumber = 0);

	/** Display text
	@param content - text
	@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void text(const char content[], uint8_t deviceNumber = 0);

	/**Test
	*/
	void test();

};


