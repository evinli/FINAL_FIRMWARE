// Linear Motor Control
// NOTE: there is a safety limit defined in this file that 
//       prevents applyong too much force on the loadcell

#include "LoadCell.h"

// TODO: Implement Acceleration Profile
//        CloseClaws(): Feedback control when gap>1cm
//        OpenClaws(): Define number of steps


// This class initialize and control the linear stepper motor
class LinearMotor {

double SAFETY_LIMIT=100; //Kg

// Movement directions
    int closeDir = 0;
    int openDir = 1;
    
    int dirCheck1 = 1;
    int dirCheck2 = 1;
    
// Step delay depends on the stepping mode    
    int stepDelay = 25; // us: Decrease to increase speed
    
    LoadCell loadcell;
    
    double appliedForce;
    double safetyMargin = 0.01;
    double loadReading;
  
  public:

   // Initializing the Linear Motor
    LinearMotor() {
      pinMode(STEP_PIN, OUTPUT);
      pinMode(DIR_PIN , OUTPUT);
      pinMode(ENABLE_PIN, OUTPUT);
      digitalWrite (ENABLE_PIN, HIGH); // Disabling the motor
    }

// This function closes the claw till the input force (in Kg) is applied

    CloseClaws(double force) {
      // Use TOF sensor for distance
      // if the gap is larger, do speed control
      int gap;
      digitalWrite(DIR_PIN, closeDir);
    
      /*
       * Need to work on this function 
       * end-stop can also be used here (testing needed)
        while (gap > 1) // gap greater than 1 cm
        {
          digitalWrite(ENABLE_PIN, LOW);
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(stepDelay);
          digitalWrite(STEP_PIN, LOW);
          delayMicroseconds(stepDelay);

          // CHECK GAP

        }
      */

      // else do step control

      
      appliedForce = loadcell.GetForce();
      digitalWrite(ENABLE_PIN, LOW);
      while (appliedForce < force) { //Need to implement a safety margin
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEP_PIN, LOW);
        //delayMicroseconds(stepDelay);
        appliedForce = loadcell.GetForce();
      }
      digitalWrite(ENABLE_PIN, HIGH);
    }

// This function opens the claw by moving the moving the
// motor an arbirary number of steps (20000, here). 
//Testing needed to find the right number
  
    void OpenClaws() {
      int numSteps;
      if (dirCheck1 == 1) {
        digitalWrite(ENABLE_PIN, LOW);
        digitalWrite(DIR_PIN, openDir);
        while (numSteps < 2000) {
          numSteps++;
            for (int num2 = 0; num2 < 10; num2++) {
              digitalWrite(STEP_PIN, HIGH);
              delayMicroseconds(stepDelay);
              digitalWrite(STEP_PIN, LOW);
              delayMicroseconds(stepDelay);
            }
        }
        digitalWrite(ENABLE_PIN, HIGH);
      }
    }
    
 // Function to get loadcell reading  from loadcell object
    double GetLoadCellReading() {
      return loadcell.GetForce();
    }

 // The function moves the linear motor [steps] number of steps in
 // the given direction [dir]
    void MoveSteps(int steps, int dir) {
      int num = 0;
      if (dir == openDir) {
        if (dirCheck1 == 1) {
          digitalWrite(DIR_PIN, dir);
          digitalWrite(ENABLE_PIN, LOW);
          while (num < steps) {
            num++;
            for (int num2 = 0; num2 < 10; num2++) {
              digitalWrite(STEP_PIN, HIGH);
              delayMicroseconds(stepDelay);
              digitalWrite(STEP_PIN, LOW);
              delayMicroseconds(stepDelay);
              
            }
          }
          digitalWrite(ENABLE_PIN, HIGH);
        }
      }
      else if (dir == closeDir) {
        if (dirCheck2 == 1) {
          digitalWrite(DIR_PIN, dir);
          digitalWrite(ENABLE_PIN, LOW);
          while (num < steps&&(loadReading<SAFETY_LIMIT)) {  // Safety Limit SET
            num++;
            for (int num2 = 0; num2 < 10; num2++) {
              digitalWrite(STEP_PIN, HIGH);
              delayMicroseconds(stepDelay);
              digitalWrite(STEP_PIN, LOW);
              delayMicroseconds(stepDelay);
              
            }
            loadReading=loadcell.GetForce();
          }
          digitalWrite(ENABLE_PIN, HIGH);
        }
      }
    }

    
    // Function for Stopping Linear Motor
    // Used by ISR when end-stop tripped
    void Disable() {
      digitalWrite (ENABLE_PIN, HIGH);
    }

    // Function for manually enabling the linear motor
    void Enable() {
      digitalWrite (ENABLE_PIN,LOW);
    }

    // Disabling/Enabling movement in a particular direction of motion
    void DisableDir1() {
      dirCheck1 = 0;
    }
    void DisableDir2() {
      dirCheck2 = 0;;
    }
    void EnableDir1() {
      dirCheck1 = 1;
    }
    void EnableDir2() {
      dirCheck2 = 1;
    }

   // Get raw data from load cell 
   double GetLoadCellvolt(){
      return loadcell.GetCal();
    }

};
