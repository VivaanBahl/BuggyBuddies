/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#include "DServo.h"
#endif
#endif

#include <cr_section_macros.h>



#define SERVO_ID 0x04               // ID of which we will set Dynamixel too
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set to (57600)
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode

//unsigned int pos = 0;

/*void SERVOINT_IRQ_HANDLER (void) {
	Chip_GPIO_ClearInts(LPC_GPIO, PIN_A_PORT, (1 << PIN_A));
}*/

int main(void) {

	SystemCoreClockUpdate(); //updates clock rate (why we need?)
	Board_Init(); //initializes board
	//Board_LED_Set(0, false);

    /*DServo.setDirectionPin(SERVO_ControlPin);

      for(int b=1; b<0xFF; b++){
        long Baudrate_BPS = 0;
        Baudrate_BPS = 2000000/(b+1);
        DServo.begin(Baudrate_BPS);
        DServo.reset(0xFE);
        for(int j = 0; j < 500000; j++){}
      }
      for(int j = 0; j < 1000000; j++){}

      DServo.begin(57600);
      DServo.setID(0xFE,SERVO_ID);
      for(int j = 0; j < 500000; j++){}
      DServo.setStatusPaket(SERVO_ID,READ);
      DServo.setBaudRate(SERVO_ID,100000);
      for(int j = 0; j < 500000; j++){}

      DServo.begin(100000);
      for(int j = 0; j < 500000; j++){}
      DServo.setMode(SERVO_ID,SERVO,0x000,0xFFF);
      for(int j = 0; j < 500000; j++){}
      //Dynamixel.setMaxTorque(SERVO_ID,0x2FF);
      //}*/

    DServo.begin(SERVO_SET_Baudrate);                                    // We now need to set Ardiuno to the new Baudrate speed
    DServo.setMode(SERVO_ID, SERVO, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE);    // set mode to SERVO and set angle limits

    for(int j = 0; j < 1000000; j++){}


    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
    	unsigned int pos = DServo.readPosition(SERVO_ID);
    	for(int j = 0; j < 5000000; j++){}
    	/*DServo.servo(SERVO_ID,0,0x100);  //  Move servo to max angle at max speed (1023)
    	for(int j = 0; j < 2000000; j++){}

    	//pos = DServo.readPosition(SERVO_ID);


    	DServo.servo(SERVO_ID,0x0FF,0x100);   // Move servo to angle 1(0.088 degree) at speed 100
    	for(int j = 0; j < 2000000; j++){}
    	//pos = DServo.readPosition(SERVO_ID);*/


        i++ ;
    }
    return 0;
}
