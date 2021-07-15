// Class for the DC motors for the Rollers

// Open Loop for now

// Rotation Definitions
#define Clock 0
#define cClock 1

class RollerMotor {
    // Speed Control
    int dirPinA;
    int dirPinB;
    int motorSpeedPin;

  public:
// This function initiates the RollerMotor object
// Parameters: pwmPIN,A,B ( See PCB Assembly instructions for more detials)
    RollerMotor(int pwmPin, int A, int B) {
      motorSpeedPin = pwmPin;
      dirPinA = A;
      dirPinB = B;
      pinMode(motorSpeedPin, OUTPUT);
      pinMode(dirPinA, OUTPUT);
      pinMode(dirPinB, OUTPUT);
    }

// This function sets the speed of the motor object
// Parameters: val: pwm duty cyle (0-100)
//             dir: clock and cClock defined at the top of this file
    void SetSpeed(int val, int dir) {
      analogWrite(motorSpeedPin, map(val, 0, 100, 0, 255));
      if (dir == Clock) {
        digitalWrite(dirPinA, LOW);
        digitalWrite(dirPinB, HIGH);
      }
      else if (dir == cClock) {
        digitalWrite(dirPinA, HIGH);
        digitalWrite(dirPinB, LOW);

      }
    }

// This function stops the motor
    void Stop() {
      digitalWrite(motorSpeedPin, 0);
    }
};
