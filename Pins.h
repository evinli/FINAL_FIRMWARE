#define __PINS__
// PIN Definitions for PCB only

// Roller Motors
#define MOTOR_PWM_PIN_LEFT 46 // pwm Capable Speed pin
#define MOTOR_A1_PIN_LEFT 47 // A1 Dir COntrol pin
#define MOTOR_B1_PIN_LEFT 48 // B1 Dir COntrol pin

#define MOTOR_PWM_PIN_RIGHT 45 // pwm Capable
#define MOTOR_A2_PIN_RIGHT 42 // A2 Dir COntrol pin
#define MOTOR_B2_PIN_RIGHT 43 // B2 Dir COntrol pin

//LoadCell
#define LOAD_CELL_OUT A0

// Linear Stepper Motor
#define STEP_PIN 13
#define DIR_PIN 12
#define ENABLE_PIN 14
#define M1 5
#define M2 6
#define M3 7

// End Stop Sensors, have to be interruptable
#define END_STOP_PIN_1 2
#define END_STOP_PIN_2 3

// Encoder Pins, have to be interruptable
#define FRONT_ENCODER_PIN 18
#define BACK_ENCODER_PIN 19

// Ultrasonic Sensor pins
#define TRIG_PIN 10
#define ECHO_PIN 9

// Extra IO
#define X1 50
#define X2 51
#define X3 52
#define X4 53	
