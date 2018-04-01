#include "DServo.h"
#include "chip.h"
#include "board.h"
#include "string.h"

/* Transmit and receive ring buffers */
STATIC RINGBUFF_T txring, rxring;

static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

void DServoClass::begin(long baud){
	//uint8_t key; //eventually declare these variables outside of begin, but for now leave it until this works
	//int bytes;

	/* Setup UART for 115.2K8N1 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT));/* RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT));/* TXD */
	Chip_UART_Init(LPC_USART); //LPC_USART is pointer to UART pin that has been initialized (?)
	Chip_UART_SetBaud(LPC_USART, baud); //set baud rate to 1 million by default (?) Currently does not set default and requires input
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT)); //configures parity bits (might need to change)
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2)); //sets up FIFOs
	Chip_UART_TXEnable(LPC_USART); //enables connection to TX (transmit)

	/* Before using the ring buffers, initialize them using the ring
		   buffer init function */
		RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
		RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);


	Chip_GPIO_SetPinDIROutput(LPC_GPIO, PIN_A_PORT, PIN_A); //Sets PIN_A to output pin


	Status_Return_Value = READ;

}

void DServoClass::setDirectionPin(unsigned char Control_ID){
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, PIN_A_PORT, Control_ID); //Sets PIN_A to output pin

}


//mode should be SERVO (set in header file to true); sets CW/CCW limits for specified servo ID
unsigned int DServoClass::setMode(unsigned char ID, bool Dynamixel_Mode, unsigned int Dynamixel_CW_Limit,unsigned int Dynamixel_CCW_Limit){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_MODE_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_CW_ANGLE_LIMIT_L;
    if (Dynamixel_Mode == WHEEL) {                                    // Set WHEEL mode, this is done by setting both the clockwise and anti-clockwise angle limits to ZERO
        Instruction_Packet_Array[4] = 0x00;
        Instruction_Packet_Array[5] = 0x00;
        Instruction_Packet_Array[6] = 0x00;
        Instruction_Packet_Array[7] = 0x00;
    }else {                                                             // Else set SERVO mode
        Instruction_Packet_Array[4] = (unsigned char)(Dynamixel_CW_Limit);
        Instruction_Packet_Array[5] = (unsigned char)((Dynamixel_CW_Limit & 0x0F00) >> 8);
        Instruction_Packet_Array[6] = (unsigned char)(Dynamixel_CCW_Limit);
        Instruction_Packet_Array[7] = (unsigned char)((Dynamixel_CCW_Limit & 0x0F00) >> 8);
    }

    /*clearRXbuffer();*/

    transmitInstructionPacket();

    if (Status_Return_Value == READ){
    readStatusPacket();
        if (Status_Packet_Array[2] != 0){
        return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }

    }

 }

unsigned int DServoClass::servo(unsigned char ID,unsigned int Position,unsigned int Speed){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SERVO_GOAL_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_GOAL_POSITION_L;
    Instruction_Packet_Array[4] = (unsigned char)(Position);
    Instruction_Packet_Array[5] = (unsigned char)((Position & 0x0F00) >> 8);
    Instruction_Packet_Array[6] = (unsigned char)(Speed);
    Instruction_Packet_Array[7] = (unsigned char)((Speed & 0x0F00) >> 8);

    /*clearRXbuffer();*/

    transmitInstructionPacket();


    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }

}

unsigned int DServoClass::readPosition(unsigned char ID){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = READ_POS_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = RAM_PRESENT_POSITION_L;
    Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;

    /*clearRXbuffer();*/

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
        return Status_Packet_Array[4] << 8 | Status_Packet_Array[3];    // Return present position value
    }else{
        return (Status_Packet_Array[2] | 0xF000);                           // If there is a error Returns error value
    }
}



unsigned int DServoClass::setBaudRate(unsigned char ID, long Baud){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_BD_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_BAUD_RATE;

    switch (Baud){
        case 1000000:
            Instruction_Packet_Array[4] = 0x01;
            break;
        case 2250000:
            Instruction_Packet_Array[4] = 0xFA;
            break;
        case 2500000:
            Instruction_Packet_Array[4] = 0xFB;
            break;
        case 3000000:
            Instruction_Packet_Array[4] = 0xFC;
            break;
        default:
        Instruction_Packet_Array[4] = (unsigned char)((2000000/Baud) - 1);
    }


    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}

unsigned int DServoClass::setID(unsigned char ID, unsigned char New_ID){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_ID_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_ID;
    Instruction_Packet_Array[4] = New_ID;

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}



unsigned int DServoClass::setStatusPaket(unsigned char  ID,unsigned char Set){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_RETURN_LEVEL_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_RETURN_LEVEL;
    Instruction_Packet_Array[4] = Set;

    Status_Return_Value = Set;

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }

}

unsigned int DServoClass::reset(unsigned char ID){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = RESET_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_RESET;

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }

}

void DServoClass::transmitInstructionPacket(void){                                   // Transmit instruction packet to Dynamixel


	bool pin_a = Chip_GPIO_GetPinState(LPC_GPIO, PIN_A_PORT, PIN_A); //gets pin state
	if (pin_a == false) //if pin is low, set state to high
	{
		Chip_GPIO_SetPinState(LPC_GPIO, PIN_A_PORT, PIN_A, true); //sets pin A to high
	}

//NEW CODE STARTS HERE

	unsigned char Header_Array[2]; //size of 2 headers
	Header_Array[0] = HEADER;
	Header_Array[1] = HEADER;

	/* send Header_Array*/
	Chip_UART_SendRB(LPC_USART, &txring, Header_Array, 2);

	/* send first three elements of Instruction_Packet_Array to txring */

	Chip_UART_SendRB(LPC_USART, &txring, Instruction_Packet_Array, 3);

	for (int i = 0; i < 10000000; i++){} //wait for information to send

	unsigned int checksum_packet = Instruction_Packet_Array[0] + Instruction_Packet_Array[1] + Instruction_Packet_Array[2]; //start reading from index 3 of instruction packet

	for (unsigned char i = 3; i <= Instruction_Packet_Array[3]; i++){
	    	Chip_UART_SendRB(LPC_USART, &txring, &Instruction_Packet_Array[i], 1);    // Write Instruction & Parameters (if there are any) to serial
	    	checksum_packet += Instruction_Packet_Array[i];
	}


    Checksum_Array[0] = ~checksum_packet & 0xFF;
	Chip_UART_SendRB(LPC_USART, &txring, Checksum_Array, 1);

//NEW CODE END

/*This is what the new code replaces (starting from here)
	Header_Array[0] = HEADER;
	Header_Array[1] = HEADER;

	for(int i = 0; i < 10; i++){
		if ((Chip_UART_ReadLineStatus(LPC_USART) & UART_LSR_RDR) != 0) break;
	}

	Chip_UART_SendBlocking(LPC_USART, Header_Array, 2); //sending the header array
	Chip_UART_SendBlocking(LPC_USART, Instruction_Packet_Array, 3); //sending the first 3 bytes in the Instruction Packet


    unsigned int checksum_packet = Instruction_Packet_Array[0] + Instruction_Packet_Array[1] + Instruction_Packet_Array[2]; //start reading from index 3 of instruction packet

    for (unsigned char i = 3; i <= Instruction_Packet_Array[1]; i++){
    	Chip_UART_SendBlocking(LPC_USART, &Instruction_Packet_Array[i], 1);    // Write Instruction & Parameters (if there are any) to serial
        checksum_packet += Instruction_Packet_Array[i];
    }

    //Chip_GPIO_DisableInt(LPC_GPIO, PIN_A_PORT, (1 << PIN_A));

    Checksum_Array[0] = ~checksum_packet & 0xFF;
    Chip_UART_SendBlocking(LPC_USART, Checksum_Array, 1); //sending the checksum array

//This is where the new code replacement ends*/

    while(UART_LSR_RDR == 0) {} //wait for TX data to be sent

    bool pin_a1 = Chip_GPIO_GetPinState(LPC_GPIO, PIN_A_PORT, PIN_A); //gets pin state
    if (pin_a1 == true) //if pin is high, set state to low
    {
    	Chip_GPIO_SetPinState(LPC_GPIO, PIN_A_PORT, PIN_A, false); //sets pin A to low
    }

    //Chip_GPIO_EnableInt(LPC_GPIO, PIN_A_PORT, (1 << PIN_A)); //enable interrupts again for pin A

}

unsigned int DServoClass::readStatusPacket(void){

    unsigned char Counter = 0x00;
    unsigned char First_Header = 0x00;
    unsigned char Temp_Status_Packet_Array[2]; //size of status pkt incl. first two header indices
    Status_Packet_Array[0] = 0x00;
    Status_Packet_Array[1] = 0x00;
    Status_Packet_Array[2] = 0x00;
    Status_Packet_Array[3] = 0x00;

    //unsigned int Time_Counter_Temp = Time_Counter + STATUS_PACKET_TIMEOUT;

    /*Time_Counter = STATUS_PACKET_TIMEOUT + millis();                                    // Setup time out error, 50 ms + millis()

    while(STATUS_FRAME_BUFFER >= _serial->available()){                                     // Wait for " header + header + frame length + error " RX data; need at
                                                                                            // least 5 indices' worth of information before we may begin to read
                                                                                            //STATUS_FRAME_BUFFER initialized to 5
                                                                                            //_serial->available() checks the number of bytes of data in RX (received)

        if (millis() >= Time_Counter){
            return Status_Packet_Array[2] = B10000000;                                      // Return with Error if Serial data not received with in time limit
        }
    }*/

    //pop off values on the receive ring buffer and stores in temp status packet array
    /*Chip_UART_ReadBlocking(LPC_USART, Temp_Status_Packet_Array, 2);

    if (Temp_Status_Packet_Array[0] == 0xFF && First_Header != 0xFF){
    	First_Header = Temp_Status_Packet_Array[0];                                                 // Clear 1st header from RX buffer (should be 0xFF)
    }else if (Temp_Status_Packet_Array[0] == -1){
    	return Status_Packet_Array[2] = 0x80;                                      // Return with Error (b10000000 = 0x80) if two headers are not found
    }

    if(Temp_Status_Packet_Array[1] == 0xFF && First_Header == 0xFF){

    	Status_Packet_Array[0] = Chip_UART_ReadBlocking(LPC_USART, Status_Packet_Array, 1);                                   // ID sent from Dynamixel
    	Status_Packet_Array[1] = Chip_UART_ReadBlocking(LPC_USART, &Status_Packet_Array[1], 1);                                   // Frame Length of status packet
    	Status_Packet_Array[2] = Chip_UART_ReadBlocking(LPC_USART, &Status_Packet_Array[2], 1);                                   // Error byte

    	do{
    		Status_Packet_Array[Counter + 3] = Chip_UART_ReadBlocking(LPC_USART, &Status_Packet_Array[Counter + 3], 1);          //set values in 3rd to nth index of Status Packet to corresponding
    																						   //values in the temporary Status Packet Array
    		Counter++;
    	}while(Status_Packet_Array[1] > Counter);                           // Read Parameter(s) into array

    	Status_Packet_Array[Counter + 4] = Chip_UART_ReadBlocking(LPC_USART, &Status_Packet_Array[Counter + 4], 1);                         // Read Check sum (trace through before verify is correct)

	}else{
		return Status_Packet_Array[2] = 0x80;                                      // Return with Error if two headers are not found
	}*/

    //pop off values on the receive ring buffer and stores in temp status packet array

    	/*char header1 = Board_UARTGetChar();
    	char header2 = Board_UARTGetChar();
    	char header3 = Board_UARTGetChar();
    	 */

    	//Chip_UART_ReadBlocking(LPC_USART, Temp_Status_Packet_Array, 2);
    	Chip_UART_ReadRB(LPC_USART, &rxring, &Temp_Status_Packet_Array, 2);

        if (Temp_Status_Packet_Array[0] == 0xFF && First_Header != 0xFF){
        	First_Header = Temp_Status_Packet_Array[0];                                                 // Clear 1st header from RX buffer (should be 0xFF)
        }else if (Temp_Status_Packet_Array[0] == -1){
        	return Status_Packet_Array[2] = 0x80;                                      // Return with Error (b10000000 = 0x80) if two headers are not found
        }

        if(Temp_Status_Packet_Array[1] == 0xFF && First_Header == 0xFF){

        	Status_Packet_Array[0] = Chip_UART_ReadRB(LPC_USART, &rxring, &Status_Packet_Array[0], 1);                                   // ID sent from Dynamixel
        	Status_Packet_Array[1] = Chip_UART_ReadRB(LPC_USART, &rxring, &Status_Packet_Array[1], 1);                                   // Frame Length of status packet
        	Status_Packet_Array[2] = Chip_UART_ReadRB(LPC_USART, &rxring, &Status_Packet_Array[2], 1);                                   // Error byte

        	do{
        		Status_Packet_Array[Counter + 3] = Chip_UART_ReadRB(LPC_USART, &rxring, &Status_Packet_Array[Counter + 3], 1);          //set values in 3rd to nth index of Status Packet to corresponding
        																						   //values in the temporary Status Packet Array
        		Counter++;
        	}while(Status_Packet_Array[1] > Counter);                           // Read Parameter(s) into array

        	Status_Packet_Array[Counter + 4] = Chip_UART_ReadRB(LPC_USART, &rxring, &Status_Packet_Array[Counter + 4], 1);                         // Read Check sum (trace through before verify is correct)

    	}else{
    		return Status_Packet_Array[2] = 0x80;                                      // Return with Error if two headers are not found
    	}
}

/*void DServoClass::clearRXbuffer(void){
	unsigned char *clearRX_Dummy_Array[14];		//initialized to size of Instruction_Packet_Array bc at most that much info can be sent
	Chip_UART_ReadBlocking(LPC_USART, clearRX_Dummy_Array, 14);


}*/

DServoClass DServo;
