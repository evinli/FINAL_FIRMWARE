// Function to control roller motors
// This class is meant to provide high level control of the roller motors
// to the main control algorithm 

#include "RollerMotor.h"
#include "Pins.h"
// TO DO: Check Pleat Loading
RollerMotor leftMotor(MOTOR_PWM_PIN_LEFT, MOTOR_A1_PIN_LEFT, MOTOR_B1_PIN_LEFT);
RollerMotor rightMotor(MOTOR_PWM_PIN_RIGHT, MOTOR_A2_PIN_RIGHT, MOTOR_B2_PIN_RIGHT);

class Gripper {
    int pleatLoaded;
    int MotorSpeed=100;
    int c = 0; //clockwise
    int cc = 1; //counter-clockwise
  
  public:
    void LoadPleat(int MotorSpeed) {

      leftMotor.SetSpeed(MotorSpeed, cc);
      rightMotor.SetSpeed(MotorSpeed, c);
      
    }
    void StartPleating(int MotorSpeed, int dir) {
      leftMotor.SetSpeed(MotorSpeed, dir);
      rightMotor.SetSpeed(MotorSpeed, dir);
    }
    void StopPleating() {
      leftMotor.Stop();
      rightMotor.Stop();
    }
    void leftMotorControl(int Motorspeed, int dir) {
      leftMotor.SetSpeed(Motorspeed, dir);
    }
    void rightMotorControl(int Motorspeed, int dir) {
      rightMotor.SetSpeed(Motorspeed, dir);
    }
    Gripper() {
      
    }
   
};
