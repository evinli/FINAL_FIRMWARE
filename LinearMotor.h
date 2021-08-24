/*  @file LinearMotor
 *   
 *  Description: This class initializes and controls the 
 *               linear stepper motor. Note that there is a
 *               safety limit defined in this file that 
 *               prevents applying too much force on the 
 *               load cell. 
 *               
 *  To-Do: Implement acceleration profile:
 *         CloseClaws(): Feedback control when force is within the optimal range 
 *         OpenClaws(): Define number of steps to open claw to desired position
 */
/* include libraries */
#include <HX711.h>

/* constants */
#define OPEN_DIR 0
#define CLOSE_DIR 1
#define NUM_STEPS 5000            /* pre-determined number for steps for opening the claw */
#define SAFETY_LIMIT 100          /* in kg, set to smaller value once we have the load cell properly calibrated */
#define STEP_DELAY 25             /* in microseconds, decrease delay to increase speed */
#define CALIBRATION_FACTOR 231000 /* calibration factor for the load cell */

class LinearMotor
{
  /* create load cell object */
  HX711 loadCell = HX711();

public:
  /* variable declarations */
  int openDirCheck = 1;  /* enable movement in opening direction */
  int closeDirCheck = 1; /* enable movement in closing direction */
  double appliedForce;

  /* initializing the linear motor and load cell */
  LinearMotor()
  {
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH); /* disable motor */

    loadCell.begin(LOAD_CELL_DATA, LOAD_CELL_CLOCK);
    loadCell.set_gain(32);
    loadCell.set_scale(CALIBRATION_FACTOR);
    //loadCell.tare(); /* tare scale to 0, but not use for serial control since reading will be tared everytime when pressing L  */
  }

  /* closes the claw until a certain input force (in kg) measured by the load cell */
  void closeClaws(double force)
  {
    digitalWrite(DIR_PIN, CLOSE_DIR);
    appliedForce = loadCell.get_units();
    digitalWrite(ENABLE_PIN, LOW);

    while (appliedForce < force && (appliedForce < SAFETY_LIMIT))
    {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(STEP_DELAY);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(STEP_DELAY);
      appliedForce = loadCell.get_units();
    }
    digitalWrite(ENABLE_PIN, HIGH);
  }

  /* opens the claw by moving the motor an arbitrary number of steps (5000, here) */
  /* further testing required to find the right number */
  void openClaws()
  {
    int numSteps = 0;
    if (openDirCheck == 1)
    {
      digitalWrite(ENABLE_PIN, LOW);
      digitalWrite(DIR_PIN, OPEN_DIR);

      while (numSteps < NUM_STEPS)
      {
        numSteps++;
        for (int i = 0; i < 10; i++)
        {
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(STEP_DELAY);
          digitalWrite(STEP_PIN, LOW);
          delayMicroseconds(STEP_DELAY);
        }
      }
      digitalWrite(ENABLE_PIN, HIGH);
    }
  }

  /* moves the linear motor [steps] number of steps in the given direction [dir] */
  void moveSteps(int steps, int dir)
  {
    int num = 0;
    if (dir == OPEN_DIR)
    {
      if (openDirCheck == 1)
      {
        digitalWrite(DIR_PIN, dir);
        digitalWrite(ENABLE_PIN, LOW);
        while (num < steps)
        {
          num++;
          for (int i = 0; i < 10; i++)
          {
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(STEP_DELAY);
            digitalWrite(STEP_PIN, LOW);
            delayMicroseconds(STEP_DELAY);
          }
        }
        digitalWrite(ENABLE_PIN, HIGH);
      }
    }
    else if (dir == CLOSE_DIR)
    {
      int num = 0;
      if (closeDirCheck == 1)
      {
        digitalWrite(DIR_PIN, dir);
        digitalWrite(ENABLE_PIN, LOW);

        while (num < steps)
        {
          num++;
          for (int i = 0; i < 10; i++)
          {
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(STEP_DELAY);
            digitalWrite(STEP_PIN, LOW);
            delayMicroseconds(STEP_DELAY);
          }
          //appliedForce = loadCell.get_units();
        }
        digitalWrite(ENABLE_PIN, HIGH);
      }
    }
  }

  /* manually disable movement in the linear motor */
  void disable()
  {
    digitalWrite(ENABLE_PIN, HIGH);
  }

  /* manually enable movement in the linear motor */
  void enable()
  {
    digitalWrite(ENABLE_PIN, LOW);
  }

  /* disables motor movement in the opening direction */
  void disableOpenDir()
  {
    openDirCheck = 0;
  }

  /* disables motor movement in the closing direction */
  void disableCloseDir()
  {
    closeDirCheck = 0;
  }

  /* enables motor movement in the opening direction */
  void enableOpenDir()
  {
    openDirCheck = 1;
  }

  /* enables motor movement in the closing direction */
  void enableCloseDir()
  {
    closeDirCheck = 1;
  }

  /* function to get calibrated force readings from load cell */
  double getLoadCellForce()
  {
    return loadCell.get_units();
  }
};
