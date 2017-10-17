/*
 * steering_driver.c
 *
 *  Created on: Oct 15, 2017
 *      Author: vivaanbahl
 */
#include <stdlib.h>
#include "board.h"

#define PWM_OFFSET_STORED_ANGLE 0
#define PWM_SCALE_STORED_ANGLE 1000 // in hundredths of a degree for precision

#define STEERING_LIMIT_LEFT -1500   //steering_set assumes that left is less than right.
#define STEERING_LIMIT_RIGHT 1500
#define STEERING_LOOP_TIME_US 10000L
#define MICROSECONDS_PER_SECOND 1000000L
#define STEERING_KP_NUMERATOR 13L
#define STEERING_KP_DEMONENATOR 1L
#define STEERING_KD_NUMERATOR 0L
#define STEERING_KD_DENOMENATOR 100L
//limited by 16-bit signed? 400 degress per second - 50% cpu usage from encoder interrupts
#define STEERING_MAX_SPEED 32000L
#define STEERING_KV_NUMERATOR 3L
#define STEERING_KV_DENOMENATOR 1000L
#define STEERING_PWM_CENTER_US 1500L //The PWM value that gives no movement.
#define STEERING_MAX_PWM 150 // max PWM value to add on top of the center_us
#define MOTOR_ENCODER_TICKS_PER_REV 36864 // 4096 * 9 = 12-bit * 1:9 gearbox
#define MOTOR_FEED_FORWARD 200
#define STEERING_ERROR_THRESHOLD 5
#define DEGREE_HUNDREDTHS_PER_REV 36000

#define SERVO_REFRESH_US 5000L
#define SERVO_MID_US 1500
#define SERVO_MIN_US 544
#define SERVO_MAX_US 2400

// TODO temp
#define TEMP_ENC_TICKS 0


#define min(X,Y) ((X < Y) ? (X) : (Y))
#define max(X,Y) ((X > Y) ? (X) : (Y))

long map_signal(long x,
                       long in_offset,
                       long in_scale,
                       long out_offset,
                       long out_scale) {
    return ((x - in_offset) * out_scale / in_scale) + out_offset;
}

long clamp(long input,
                  long upper,
                  long lower) {
    return (max(min(input, upper), lower));
}

void servo_set_us(uint16_t value)
{
	// check limits
	if(value < SERVO_MIN_US) {
		value = SERVO_MIN_US;
	}
	if(value > SERVO_MAX_US) {
		value = SERVO_MAX_US;
	}

	LPC_TIMER16_0->MR[0] = SERVO_REFRESH_US - value;
}

/** @brief sets the velocity of the steering motor to target_velocity
 *
 *  @param target_velocity is in hundredths of a degree per second. values > 0
 *      drive to the right
 */
void steer_set_velocity(long target_velocity) {
    static long steer_set_prev_ticks = 0;
    static long steer_set_prev_velocity = 0;

    target_velocity = clamp(target_velocity, STEERING_MAX_SPEED, -(STEERING_MAX_SPEED));

    long current_ticks = TEMP_ENC_TICKS;
    long change_in_ticks = current_ticks - steer_set_prev_ticks;
    long change_in_angle = (change_in_ticks * DEGREE_HUNDREDTHS_PER_REV) / DEGREE_HUNDREDTHS_PER_REV;
    //angle * us/s * us = angle per second
    long actual_velocity = (change_in_angle * MICROSECONDS_PER_SECOND) / STEERING_LOOP_TIME_US;

    long error = target_velocity - actual_velocity;

    long output_us = 0;
    long output_p = error * STEERING_KV_NUMERATOR / STEERING_KV_DENOMENATOR;
    output_us = output_p;
    long output_int_us = steer_set_prev_velocity + output_us;
    output_int_us = clamp(output_int_us, STEERING_MAX_PWM, -STEERING_MAX_PWM);

    // dbg_printf("target: %ld, current: %ld, error: %ld, correction: %ld, output: %ld\n",
    // target_velocity, actual_velocity, error, output_us, output_int_us);

    //Send command to the motor
    servo_set_us(output_int_us + STEERING_PWM_CENTER_US);

    steer_set_prev_ticks = current_ticks;
    steer_set_prev_velocity = output_int_us;
}


/** @brief sets the steering angle using PID feedback
 *
 *  @param angle set point in hundredths of a degree
 */
void steering_set(int angle)
{
    static long steer_set_error_prev = 0; //This is used to find the d term for position.

    angle = clamp(angle, STEERING_LIMIT_RIGHT, STEERING_LIMIT_LEFT);

    //For the DC motor
    long actual = map_signal(TEMP_ENC_TICKS,
                             0,
                             MOTOR_ENCODER_TICKS_PER_REV,
                             0,
                             DEGREE_HUNDREDTHS_PER_REV);

    long error = angle - actual;
    long output_vel = 0;
    if(labs(error) > STEERING_ERROR_THRESHOLD) { //0.1 degree deadband
        long output_p = (STEERING_KP_NUMERATOR * error) / STEERING_KP_DEMONENATOR;
        long output_ff = (error > 0) ? MOTOR_FEED_FORWARD : -MOTOR_FEED_FORWARD;
        long d =  (error - steer_set_error_prev); //not dividing by time
        long output_d = (STEERING_KD_NUMERATOR * d) / STEERING_KD_DENOMENATOR;
        output_vel = output_p + output_ff + output_d;
    }

    steer_set_error_prev = error;
    steer_set_velocity(output_vel);
}
