/*  @file Pins
 *   
 *  Description: This class defines pins used by various sensors   
 *               on the Arduino board.
 */

#ifndef __PINS__
#define __PINS__

/* roller motors */
#define MOTOR_PWM_PIN_LEFT 46
#define MOTOR_A1_PIN_LEFT 47
#define MOTOR_B1_PIN_LEFT 48

#define MOTOR_PWM_PIN_RIGHT 45
#define MOTOR_A2_PIN_RIGHT 42
#define MOTOR_B2_PIN_RIGHT 43

/* linear stepper motor */
#define STEP_PIN 13
#define DIR_PIN 12
#define ENABLE_PIN 14
#define M1 5
#define M2 6
#define M3 7

/* load cell */
#define LOAD_CELL_CLOCK A0
#define LOAD_CELL_DATA A1

/* end-stop sensor, have to be interruptable */
#define ENDSTOP_PIN 2

/* encoder pins, have to be interruptable */
#define FRONT_ENCODER_PIN 18
#define BACK_ENCODER_PIN 19

/* ultrasonic sensor */
#define TRIG_PIN 10
#define ECHO_PIN 9

/* extra IO pins */
#define X1 50
#define X2 51
#define X3 52
#define X4 53
#endif
