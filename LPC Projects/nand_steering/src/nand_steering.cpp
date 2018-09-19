/*
===============================================================================
 Name        : main.c
 Author      : Adam Quinn & Tiffany Yu
 Version     :
 Copyright   : 2018
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#include "DServo.h"
#include "easyTimer.h"
#endif
#endif

#include <cr_section_macros.h>



#define SERVO_ID 0xFE               //Servo ID used for sending movement commands.
#define SERVO_RECEIVE_ID 0x04		//Servo ID used for sending commands for which we expect a response.
#define TRISTATE_TOGGLE_PORT 2
#define TRISTATE_TOGGLE_PIN 0
#define DYNAMIXEL_BAUD 57600
#define CW_LIMIT_ANGLE 1950       //Limit angles determined by the construction of the steering system.
#define CCW_LIMIT_ANGLE 2550

#define NEUTRAL_ANGLE 2250
#define DEFAULT_SPEED 0x100


int main(void) {

	SystemCoreClockUpdate(); //updates clock rate (why we need?)
	Board_Init(); //initializes board

	ET.InitEasyTimer();

    DServo.setDirectionPin(TRISTATE_TOGGLE_PORT,TRISTATE_TOGGLE_PIN);

    DServo.begin(DYNAMIXEL_BAUD);                                    // We now need to set Ardiuno to the new Baudrate speed
    DServo.setMode(SERVO_ID, SERVO, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE);    // set mode to SERVO and set angle limits
    DServo.setID(SERVO_ID, SERVO_RECEIVE_ID);
    DServo.servo(SERVO_ID,NEUTRAL_ANGLE,DEFAULT_SPEED);
    DServo.setStatusPaket(SERVO_ID,ALL);
    ET.delay(1000);


    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
    	ET.delay(1000);
    	DServo.servo(SERVO_ID,2000,DEFAULT_SPEED);
    	ET.delay(1000);
    	DServo.servo(SERVO_ID,2500,DEFAULT_SPEED);   // Move servo to angle 1(0.088 degree) at speed 100

    	ET.delay(500);
    	unsigned int pos = DServo.readPosition(SERVO_RECEIVE_ID);


        i++ ;
    }

    DServo.deconstructor();
    return 0;
}
