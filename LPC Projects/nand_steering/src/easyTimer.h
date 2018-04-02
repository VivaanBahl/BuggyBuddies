/*
 * easyTimer.h
 *
 *  Created on: Mar 26, 2018
 * 	  Modified:	Apr 2, 2018
 *      Author: Adam Quinn
 */


#ifndef EASYTIMER_H_
#define EASYTIMER_H_

#include <stdint.h>

class easyTimer{

private:
	uint32_t currentMillis;
	bool initialized = false;

public:
	uint32_t millis();
	void delay(uint32_t milliseconds);
	void InitEasyTimer();
	void increment();
};

extern easyTimer ET;

#endif /* EASYTIMER_H_ */
