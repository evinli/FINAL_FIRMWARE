/*  @file RollerMotor 
 *   
 *  Description: This class initializes and controls the 
 *               DC motors for the rollers.     
 */

/* constants */
#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1

class RollerMotor
{
public:
  /* variable declarations */
  int dirPinA;
  int dirPinB;
  int motorSpeedPin;

  /* intializing the DC motor */
  RollerMotor(int pwmPin, int A, int B)
  {
    motorSpeedPin = pwmPin;
    dirPinA = A;
    dirPinB = B;
    pinMode(motorSpeedPin, OUTPUT);
    pinMode(dirPinA, OUTPUT);
    pinMode(dirPinB, OUTPUT);
  }

  /* sets speed for the DC motor */
  void setSpeed(int val, int dir)
  {
    analogWrite(motorSpeedPin, map(val, 0, 100, 0, 255)); /* map PWM duty cycle (0-100) to a PWM speed value from 0-255 */
    if (dir == CLOCKWISE)
    {
      digitalWrite(dirPinA, LOW);
      digitalWrite(dirPinB, HIGH);
    }
    else if (dir == COUNTER_CLOCKWISE)
    {
      digitalWrite(dirPinA, HIGH);
      digitalWrite(dirPinB, LOW);
    }
  }

  /* stops rollers by setting DC motor speed to 0 */
  void stop()
  {
    digitalWrite(motorSpeedPin, 0);
  }
};
