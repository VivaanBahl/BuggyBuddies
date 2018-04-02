#ifndef DServo_h
#define DServo_h
#define PIN_A					2 /* GPIO pin number mapped to PIN_A*/
#define PIN_A_PORT				0 /* GPIO port number mapped to PIN_A */
#define IOCON_PIN_A_ID			IOCON_PIO0_2 /* IOCON pin identifer */
#define SERVOINT_IRQ_HANDLER	PIOINT0_IRQHandler /* t1: GPIO interrupt IRQ function name */
#define SERVOINT_NVIC_NAME		EINT0_IRQn /* t1: GPIO interrupt NVIC interrupt name */


/*Move PIN_A thru SERVOINT_NVIC_NAME to table below*/

//#########################################################################
//################ define - Dynamixel Hex code table ######################
// EEPROM AREA
#define EEPROM_MODEL_NUMBER_L           0x00
#define EEPROM_MODEL_NUMBER_H           0x01
#define EEPROM_VERSION                  0x02
#define EEPROM_ID                       0x03
#define EEPROM_BAUD_RATE                0x04
#define EEPROM_RETURN_DELAY_TIME        0x05
#define EEPROM_CW_ANGLE_LIMIT_L         0x06
#define EEPROM_CW_ANGLE_LIMIT_H         0x07
#define EEPROM_CCW_ANGLE_LIMIT_L        0x08
#define EEPROM_CCW_ANGLE_LIMIT_H        0x09
#define EEPROM_LIMIT_TEMPERATURE        0x0B
#define EEPROM_LOW_LIMIT_VOLTAGE        0x0C
#define EEPROM_HIGN_LIMIT_VOLTAGE       0x0D
#define EEPROM_MAX_TORQUE_L             0x0E
#define EEPROM_MAX_TORQUE_H             0x0F
#define EEPROM_RETURN_LEVEL             0x10
#define EEPROM_ALARM_LED                0x11
#define EEPROM_ALARM_SHUTDOWN           0x12
// RAM AREA
#define RAM_TORQUE_ENABLE               0x18
#define RAM_LED                         0x19
#define RAM_PROPORTIONAL_GAIN           0x1C
#define RAM_INTERGRAL_GAIN              0x1B
#define RAM_DERIVATIVE_GAIN             0x1A
#define RAM_GOAL_POSITION_L             0x1E
#define RAM_GOAL_POSITION_H             0x1F
#define RAM_GOAL_SPEED_L                0x20
#define RAM_GOAL_SPEED_H                0x21
#define RAM_TORQUE_LIMIT_L              0x22
#define RAM_TORQUE_LIMIT_H              0x23
#define RAM_PRESENT_POSITION_L          0x24
#define RAM_PRESENT_POSITION_H          0x25
#define RAM_PRESENT_SPEED_L             0x26
#define RAM_PRESENT_SPEED_H             0x27
#define RAM_PRESENT_LOAD_L              0x28
#define RAM_PRESENT_LOAD_H              0x29
#define RAM_PRESENT_VOLTAGE             0x2A
#define RAM_PRESENT_TEMPERATURE         0x2B
#define RAM_REGISTER                    0x2C
#define RAM_MOVING                      0x2E
#define RAM_LOCK                        0x2F
#define RAM_PUNCH_L                     0x30
#define RAM_PUNCH_H                     0x31


//#########################################################################
//################ Instruction commands Set ###############################
#define COMMAND_PING                    0x01
#define COMMAND_READ_DATA               0x02
#define COMMAND_WRITE_DATA              0x03
#define COMMAND_REG_WRITE_DATA          0x04
#define COMMAND_ACTION                  0x05
#define COMMAND_RESET                   0x06
#define COMMAND_SYNC_WRITE              0x83


//#########################################################################
//################ Instruction packet lengths #############################
// Packet length is number of parameters (N) + 2
#define READ_ONE_BYTE_LENGTH            0x01
#define READ_TWO_BYTE_LENGTH            0x02
#define RESET_LENGTH                    0x02
#define PING_LENGTH                     0x02
#define ACTION_LENGTH                   0x02
#define SET_ID_LENGTH                   0x04
#define SET_BD_LENGTH                   0x04
#define SET_RETURN_LEVEL_LENGTH         0x04
#define READ_TEMP_LENGTH                0x04
#define READ_POS_LENGTH                 0x04
#define READ_LOAD_LENGTH                0x04
#define READ_SPEED_LENGTH               0x04
#define READ_VOLT_LENGTH                0x04
#define READ_REGISTER_LENGTH            0x04
#define READ_MOVING_LENGTH              0x04
#define READ_LOCK_LENGTH                0x04
#define LED_LENGTH                      0x04
#define SET_HOLDING_TORQUE_LENGTH       0x04
#define SET_MAX_TORQUE_LENGTH           0x05
#define SET_ALARM_LENGTH                0x04
#define READ_LOAD_LENGTH                0x04
#define SET_RETURN_LENGTH               0x04
#define WHEEL_LENGTH                    0x05
#define SERVO_GOAL_LENGTH               0x07
#define SET_MODE_LENGTH                 0x07
#define SET_PUNCH_LENGTH                0x04
#define SET_PID_LENGTH                  0x06
#define SET_TEMP_LENGTH                 0x04
#define SET_VOLT_LENGTH                 0x05
#define SYNC_LOAD_LENGTH                0x0D
#define SYNC_DATA_LENGTH                0x02


//#########################################################################
//############################ Specials ###################################
#define PORT0                           0x00
#define PORT1                           0x01
#define PORT2                           0x02
#define PORT3                           0x03

#define OFF                             0x00
#define ON                              0x01

#define SERVO                           0x01
#define WHEEL                           0x00

#define LEFT                            0x00
#define RIGHT                           0x01

#define NONE                            0x00
#define READ                            0x01
#define ALL                             0x02

#define BROADCAST_ID                    0xFE

#define HEADER                          0xFF

#define STATUS_PACKET_TIMEOUT           50      // in millis()
#define STATUS_FRAME_BUFFER             5

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 128	/* Send */
#define UART_RRB_SIZE 32	/* Receive */

class DServoClass {
	private:
		void transmitInstructionPacket(void);
	    unsigned int readStatusPacket(void);
	    /*void clearRXbuffer(void);*/

	    //LPC_USART_T *LPC_USART; //<--this is the name of RX/TX to set up

	    unsigned char   Instruction_Packet_Array[14];   // Array to hold instruction packet data
	    unsigned char   Status_Packet_Array[8];         // Array to hold returned status packet data
	    unsigned long   Time_Counter;                   // Timer for time out watchers
	    char            Direction_Pin;                  // Pin to control TX/RX buffer chip
	    unsigned char   Status_Return_Value;            // Status packet return states ( NON , READ , ALL ); default should be ALL

	    unsigned char Header_Array[2];					//Array to hold HEADER data (0xFF for both indices)
	    unsigned char Checksum_Array[1];				//Array to hold low bit of checksum (in TransmitInstructionPacket)

	protected:

	public:
	    void begin(long baud);
	    void setDirectionPin(unsigned char);
	    void deconstructor();

        //unsigned int Time_Counter;    //current time in milliseconds

		unsigned int setBaudRate(unsigned char, long);
		unsigned int setMode(unsigned char, bool, unsigned int, unsigned int);
		unsigned int servo(unsigned char ID,unsigned int Position,unsigned int Speed);
		unsigned int readPosition(unsigned char ID);
		unsigned int setID(unsigned char ID, unsigned char New_ID);
		unsigned int setStatusPaket(unsigned char  ID,unsigned char Set);
		unsigned int reset(unsigned char ID);
		void SERVO_IRQ_HANDLER(void); //t1:added to handle interrupts (Do we need?)


};

extern DServoClass DServo;

#endif

