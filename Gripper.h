/*  @file Gripper
 *   
 *  Description: This class provides high-level control of      
 *               pleating tasks (loading & migration) to the 
 *               main control alorithm.           
 *               
 *  To-Do: Ideally this class would have two main functions, 
 *         loadPleat() and migratePleat(), each containing smaller
 *         lower-level helper functions like closeClaw(), rollerLoad(), 
 *         etc that rely on feedback from the load cell, ToF sensor 
 *         array, and rotary encoders. 
 */

/* include libraries */
#include "RollerMotor.h"
#include "Pins.h"

/* constants */
#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1


class Gripper {
  RollerMotor rightMotor = RollerMotor(MOTOR_PWM_PIN_RIGHT, MOTOR_A1_PIN_RIGHT, MOTOR_B1_PIN_RIGHT);
  RollerMotor leftMotor = RollerMotor(MOTOR_PWM_PIN_LEFT, MOTOR_A2_PIN_LEFT, MOTOR_B2_PIN_LEFT);
    
  public:
    /* initializing the gripper */
    Gripper()
    {
      /* will add to this in the future */
    }
    
    void loadPleat(int motorSpeed)
    {
      /* need to update, it's a lot more advanced than this */
      leftMotor.setSpeed(motorSpeed, COUNTER_CLOCKWISE);
      rightMotor.setSpeed(motorSpeed, CLOCKWISE);
    }
    void migratePleat(int motorSpeed, int dir)
    {
      /* need to update, it's a lot more advanced than this */
      leftMotor.setSpeed(motorSpeed, dir);
      rightMotor.setSpeed(motorSpeed, dir);
    }
    void stopPleating()
    {
      /* need to update, it's a lot more advanced than this */
      leftMotor.stop();
      rightMotor.stop();
    }
  
    /* controlling left DC motor */
    void leftMotorControl(int motorSpeed, int dir)
    {
      leftMotor.setSpeed(motorSpeed, dir);
    }
  
    /* controlling right DC motor */
    void rightMotorControl(int motorSpeed, int dir)
    {
      rightMotor.setSpeed(motorSpeed, dir);
    }
};
