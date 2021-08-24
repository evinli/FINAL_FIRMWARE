/*  @file FINAL_FIRMWARE 
 *   
 *  Description: Control code for the VIMEE Gripper, this code 
 *               reads sensor data and performs basic feedback 
 *               control.
 * 
 *  Included libraries: Gripper.h
 *                      LinearMotor.h
 *                      LoadCell.h
 *                      Pins.h
 *                      RollerMotor.h
 *  
 *  Note: A ToF sensor file is not included since the I2C library
 *        does not work inside a class, and thus must be inside the 
 *        main program file. Only the rightmost end-stop (referred 
 *        to as 'end-stop') is actively used; linear motor is disabled 
 *        in the closing direction when a predetermined grip force is 
 *        measured by the load cell. 
 */

/* include libraries */
#include "Gripper.h"
#include "LinearMotor.h"
#include "Wire.h"
#include <Adafruit_VL6180X.h>

/* constants */
#define TOF_THRESHOLD 60 /* min. distance (in mm) the pleat needs to be in order to trigger the ToF sensor */
#define NUM_SENSORS 3
#define TOF1_CHANNEL 0
#define TOF2_CHANNEL 1
#define TOF3_CHANNEL 2
#define HEIGHT0 0 /* arbitrary baseline pleat height */
#define HEIGHT1 5 /* arbitrary pleat height inside the claw */
#define HEIGHT2 10
#define HEIGHT3 15
#define TCAADDR 0x70 /* define I2C address for multiplexer */
#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define OPEN_DIR 0
#define CLOSE_DIR 1
#define ROLLER_SPEED_DEFAULT 100 /* 100% PWM duty cycle */
#define LOADING_FORCE 1.8 /* lower limit of optimal range for pleat loading */
#define MIGRATING_FORCE 0.4 /* lower limit of optimal range for pleat migration */

/* create three new ToF sensor objects */
Adafruit_VL6180X ToF1 = Adafruit_VL6180X();
Adafruit_VL6180X ToF2 = Adafruit_VL6180X();
Adafruit_VL6180X ToF3 = Adafruit_VL6180X();

/* create new gripper and LMotor objects */
Gripper gripper = Gripper();
LinearMotor LMotor = LinearMotor();

/* variable declarations */
int sensorReadings[NUM_SENSORS]; /* for storing ToF sensor array readings */
int currentIndex = HEIGHT0; /* indexes refer to ToF sensors 1, 2, 3 respectively, each representing an arbitrary height */
int currentHeight; /* for storing current pleat height */
int newHeight; /* for storing new pleat height */
int pleatSlip; /* 0 or 1 for detecting whether pleat has slipped */

byte serialData; /* for storing commands from the serial monitor */

/* not in-use, encoders haven't been integrated */
//volatile unsigned long current;
//volatile unsigned long diff;
//volatile unsigned long old;
//volatile bool check = LOW;
//int timePeriod = 100; /* in ms */

/* empty variables, each of a different data type */
String readString;
char c;
int n;
double l;

volatile int endstopTripped = 0;
double duration; /* stores sound wave reflection time */
double distance; /* stores distance from ultrasonic sensor */

void setup()
{
  Serial.begin(9600);
  Wire.begin(); /* enable I2C protocol */
  analogReference(EXTERNAL); /* sets the ADC reference voltage to 2.5 V */

  /* initialize endstop pins */
  pinMode(ENDSTOP_PIN, INPUT);

  /* initialize ultrasonic sensor pins */
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  /* attach interrupt to right end-stop signal */
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_PIN), ISR_ENDSTOP, CHANGE);

  /* not in-use since encoder hasn't been integrated */
  //  /* interrupt for the encoder disk sensor that is already on the gripper */
  //  /* TO-DO: Add interrupt for the second encoder disk */
  //  pinMode(FRONT_ENCODER_PIN, INPUT);
  //  attachInterrupt(digitalPinToInterrupt(FRONT_ENCODER_PIN), TimePeriodMeasure, RISING);

  /* starting initialization */
  Serial.println("##############################");
  Serial.println("Starting Initialization");
  Serial.println("##############################");

  /* initializing 1st ToF sensor */
  tcaselect(TOF1_CHANNEL);
  if (!ToF1.begin())
  {
    Serial.print("ToF1 detected?\t");
    Serial.print("No");
    Serial.println();
  }
  else
  {
    Serial.print("ToF1 detected?\t");
    Serial.print("Yes");
    Serial.println();
  }

  /* initializing 2nd ToF sensor */
  tcaselect(TOF2_CHANNEL);
  if (!ToF2.begin())
  {
    Serial.print("ToF2 detected?\t");
    Serial.print("No");
    Serial.println();
  }
  else
  {
    Serial.print("ToF2 detected?\t");
    Serial.print("Yes");
    Serial.println();
  }

  /* initializing 3rd ToF sensor */
  tcaselect(TOF3_CHANNEL);
  if (!ToF3.begin())
  {
    Serial.print("ToF3 detected?\t");
    Serial.print("No");
    Serial.println();
  }
  else
  {
    Serial.print("ToF3 detected?\t");
    Serial.print("Yes");
    Serial.println();
  }

  /* finishing initialization */
  Serial.println();
  Serial.println("##############################");
  Serial.println("Initialization Finished");
  Serial.println("##############################");
  Serial.println();

  /* command definitions */
  Serial.println("\nREADY TO ACCEPT COMMANDS");

  /* unused commands - for closed-loop automation */
  //  Serial.println("Press a for automatic mode.....");
  //  Serial.println("l: loading pleat up");
  //  Serial.println("s: migrating pleating"); // need to change
  //  Serial.println("q: quitting automatic mode"):

  /* DC roller motor commands */
  Serial.println("\nDC ROLLER MOTOR COMMANDS");
  /* Enter z100 for 100% duty cycle, z50 for 50% duty cycle and so on */
  Serial.println("z: left motor clockwise, eg. z100");
  /* Enter x100 for 100% duty cycle, x50 for 50% and so on */
  Serial.println("x: left motor cClockwise, eg. x100");
  /* Enter .100 for 100% duty cycle, .50 for 50% duty cycle and so on */
  Serial.println(",: right motor clockwise, eg. ,100");
  /* Enter /100 for 100% duty cycle, /50 for 50% duty cycle and so on */
  Serial.println("/: right motor cClockwise, eg. /100");

  /* linear motor commands */
  Serial.println("\nLINEAR MOTOR COMMANDS");
  /* Enter [5000 for moving 5000 steps forward and so on */
  Serial.println("[: linear motor open. eg. [5000");
  /* Enter ]5000 for moving 5000 steps backward and so on */
  Serial.println("]: linear motor close, eg. ]5000");

  /* sensor output */
  Serial.println("\nSENSOR OUTPUT COMMANDS");
  Serial.println("L: print load cell force readings continously");
  Serial.println("V: print ToF sensor trigger status continously");
  Serial.println("v: print pleat height continously");
  Serial.println("T: encoder1 period output continously"); /* come back to this, left or right encoder??? */
  Serial.println("u: ultrasonic sensor output continously");

  /* useful open-loop control commands */
  Serial.println("\nUSEFUL OPEN-LOOP CONTROL COMMANDS");
  /* I: run right motor counter-clockwise and left motor clockwise (pushing pleat out @100% duty cycle) */
  Serial.println("I: rotate rollers to unload pleat out of gripper");
  /* i: run right motor clockwise and left motor counter-clockwise (pulling pleat up @100% duty cycle) */
  Serial.println("i: rotate rollers to load pleat into gripper");
  /* O: run both motors counter-clockwise @100% duty cycle */
  Serial.println("O: rotate both rollers counter-clockwise for pleat migration");
  /* o: run both motors clockwise @100% duty cycle */
  Serial.println("o: rotate both rollers clockwise for pleat migration");
  /* c[force]: close claw until a pre-determined force in kg [force] is applied */
  Serial.println("c: close claw until a user-defined force in kg is applied, eg. c0.9 (for 0.9 kg)");
  /* h: open claw to release pleat, currently set to a default of 5000 steps */
  Serial.println("h: open claws");
  /* S: stop roller motors */
  Serial.println("S: stop roller motors");
}

void loop()
{
  if (Serial.available() > 0)
  {
    serialData = Serial.read();
  }

  /* notify user if right end-stop has been tripped */
  if (endstopTripped == 1)
    Serial.println("End-stop has been tripped...");

  /* if-else commands explained in set-up function*/
  if (serialData == 'z')
  {
    /* getting the duty cycle from the serial buffer to determine motor speed */
    while (!Serial.available())
      ;
    while (Serial.available())
    {
      c = Serial.read(); /* reads one character from serial buffer */
      readString += c; /* appends character to a string */
      delay(10); /* slow looping to allow buffer to fill with next character */
    }
    n = readString.toInt(); /* converts string to an integer value */
    gripper.leftMotorControl(n, CLOCKWISE);
    readString = ""; /* clear readString variable */
  }
  else if (serialData == 'x')
  {
    while (!Serial.available())
      ;
    while (Serial.available())
    {
      c = Serial.read();
      readString += c;
      delay(10);
    }
    n = readString.toInt();
    gripper.leftMotorControl(n, COUNTER_CLOCKWISE);
    readString = "";
  }
  else if (serialData == ',')
  {
    while (!Serial.available())
      ;
    while (Serial.available())
    {
      c = Serial.read();
      readString += c;
      delay(10);
    }
    n = readString.toInt();
    gripper.rightMotorControl(n, CLOCKWISE);
    readString = "";
  }
  else if (serialData == '/')
  {
    while (!Serial.available())
      ;
    while (Serial.available())
    {
      c = Serial.read();
      readString += c;
      delay(10);
    }
    n = readString.toInt();
    gripper.rightMotorControl(n, COUNTER_CLOCKWISE);
    readString = "";
  }
  else if (serialData == '[')
  {
    /* getting the number of steps from the serial buffer for the linear motor */
    while (!Serial.available());
    while (Serial.available())
    {
      c = Serial.read();
      readString += c;
      delay(10);
    }
    n = readString.toInt();
    LMotor.moveSteps(n, OPEN_DIR);
    readString = "";
  }
  else if (serialData == ']')
  {
    while (!Serial.available());
    while (Serial.available())
    {
      c = Serial.read();
      readString += c;
      delay(10);
    }
    n = readString.toInt();
    LMotor.moveSteps(n, CLOSE_DIR);
    readString = "";
  }
  else if (serialData == 'L')
  {
    while (1)
    {
      Serial.print("Force reading (in kg): ");
      Serial.print(LMotor.getLoadCellForce());
      Serial.println();
    }
  }
  else if (serialData == 'V')
  {
    readSensors();
    while (1)
    {
      readSensors();
      for (int i = 0; i < NUM_SENSORS; i++)
      {
        Serial.print("ToF ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(sensorReadings[i]);
        Serial.println();
      }
      delay(300);
      Serial.println();
    }
  }
  else if (serialData == 'v')
  {
    currentHeight = getPleatHeight();
    while (1)
    {
      newHeight = getPleatHeight();
      pleatSlip = checkPleatSlipping(currentHeight, newHeight);
      Serial.print("Pleat height: ");
      Serial.print(newHeight);
      Serial.println();
      if (pleatSlip == 1)
      {
        Serial.print("Pleat has slipped - please reload.");
        Serial.end();
      }
      currentHeight = newHeight;
    }
  }
  /* not in-use - encoder hasn't been integrated yet */
//  else if (serialData == 'T') {
//    while (1) {
//      if (current > 0) {
//        Serial.println(current);
//        delay(100);
//      }
//    }
//  }
  else if (serialData == 'u')
  {
    while (1)
    {
      digitalWrite(TRIG_PIN, LOW); /* clears the trigPin */
      delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH); /* sets the trigPin on HIGH state for 10 microseconds */
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW); /* pull the trigPin back to LOW state */
      duration = pulseIn(ECHO_PIN, HIGH); /*reads the echoPin, returns the sound wave travel time in microseconds */
      distance = duration * 0.034 / 2; /* calculates distance in cm */
      Serial.println(distance);
      delay(100);
    }
  }
  else if (serialData == 'I')
  {
    gripper.rightMotorControl(5, COUNTER_CLOCKWISE);
    gripper.leftMotorControl(5, CLOCKWISE);
  }
  else if (serialData == 'i')
  {
    gripper.rightMotorControl(100, CLOCKWISE);
    gripper.leftMotorControl(100, COUNTER_CLOCKWISE);
  }
  else if (serialData == 'O')
  {
    gripper.rightMotorControl(100, COUNTER_CLOCKWISE);
    gripper.leftMotorControl(100, COUNTER_CLOCKWISE);
  }
  else if (serialData == 'o')
  {
    gripper.rightMotorControl(100, CLOCKWISE);
    gripper.leftMotorControl(100, CLOCKWISE);
  }
  else if (serialData == 'S')
  {
    gripper.rightMotorControl(0, CLOCKWISE);
    gripper.leftMotorControl(0, COUNTER_CLOCKWISE);
  }
  else if (serialData == 'c')
  {
    /* getting force from the serial buffer to determine how much to close the claw */
    while (!Serial.available())
      ;
    while (Serial.available())
    {
      c = Serial.read();
      readString += c;
      delay(10);
    }
    l = readString.toDouble();
    LMotor.closeClaws(l);
    readString = "";
  }
  else if (serialData == 'h')
  {
    LMotor.openClaws();
  }

  /* for closed-loop control - not currently used */
  //  else if (serialData == 'l') {
  //    gripper.loadPleat(MotorSpeed);
  //  }
  //  else if (serialData == 's') {
  //    gripper.startPleating(MotorSpeed, PleatingDir);
  //  }
  //  else if (serialData == 'P') {
  //    LMotor.CloseClaws(0.1);
  //    gripper.LoadPleat(100);
  //    delay(1000);
  //    CheckPleatLoading();
  //    delay(2000);
  //    LMotor.moveSteps(350, 1);
  //    gripper.rightMotorControl(100, 1);
  //    gripper.leftMotorControl(100, 1);
  //    readSensors();
  //    for(int counter=NUM_SENSORS-2; counter>=0; counter--){
  //      Serial.print(" Sensor ");
  //      Serial.print(counter);
  //      Serial.print(" ");
  //      Serial.println(sensorReadings[counter]);
  //    }
  //  }

  serialData = 0;
}

/**********************************/
/*** TOF SENSOR ARRAY FUNCTIONS ***/
/* enables multiplexer to toggle between each ToF sensor */
void tcaselect(uint8_t bus)
{
  if (bus > 7)
    return;

  Wire.beginTransmission(TCAADDR); /* transmit to multiplexer */
  Wire.write(1 << bus); /* send byte to select bus (0 - 7) */
  Wire.endTransmission();
}

/* reads distance detected by one ToF sensor */
int readSensor(int sensorNum)
{
  tcaselect(sensorNum);
  if (sensorNum = TOF1_CHANNEL)
    return ToF1.readRange();
  else if (sensorNum = TOF2_CHANNEL)
    return ToF2.readRange();
  else if (sensorNum = TOF3_CHANNEL)
    return ToF3.readRange();
  else
    return 0;
}

/* reads all ToF sensors and returns values to an array */
/* returns 1 if sensor is tripped, 0 otherwise */
void readSensors()
{
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (readSensor(i) <= TOF_THRESHOLD)
    {
      sensorReadings[i] = 1;
    }
    else
    {
      sensorReadings[i] = 0;
    }
  }
}

/* returns pleat height in terms of last ToF sensor tripped */
int getPleatHeight()
{
  int counter = 0;
  currentIndex = counter;
  readSensors(); /* get updated readings for each sensor */

  while ((sensorReadings[counter] == 1) && (counter <= 2))
  {
    currentIndex = counter + 1;
    counter++;
    readSensors();
  }

  if (currentIndex == 1)
    return HEIGHT1;
  else if (currentIndex == 2)
    return HEIGHT2;
  else if (currentIndex == 3)
    return HEIGHT3;
  else
    return HEIGHT0; /* pleat has yet to trigger any sensor */
}

/* checks if pleat is slipping; returns 1 if slipping, 0 otherwise */
int checkPleatSlipping(int currentHeight, int newHeight)
{
  if (newHeight < currentHeight)
    return 1;
  else
    return 0;
}

/**********************************/
/*** END-STOP FUNCTIONS ***/
/* interrupt service routine (ISR) for when the right end-stop is tripped */
void ISR_ENDSTOP()
{
  /* when the end-stop is triggered (output = HIGH), disable further movement in that direction */
  if (digitalRead(ENDSTOP_PIN) == HIGH)
  {
    endstopTripped = 1;
    /* disable linear motors */
    LMotor.disable();
    LMotor.disableOpenDir();
  }
  else
  {
    /* when end-stop is cleared, enable further movement in that direction */
    endstopTripped = 0;
    LMotor.enable();
    LMotor.enableOpenDir();
  }
}

/**********************************/
/* ENCODER FUNCTIONS */
/*
// * ISR Function for encoder 1:
// * This function measure the time between the rising edges of the square wave signal 
//   of the encoder sensor (Time period of the signal)
// * e.g. If the encoder signal is 10 Hz square wave, current will be 1/10Hz = 100ms
//   (i.e. the time period of signal)
// 
// NOTE: A second microcontroller was used to do the this function beacause of a problem
// with the protoboard so THIS FUNCTION IS NOT USED IN THE CURRENT CODE. Refer to code for
// the secondary microcontroller.
// TO DO: Use this function with main microcontroller instead of secondary micrcontoller 
//        along with the CheckPleatLoading()
//        (Eliminate the need to the second microcontroller
// */
//void TimePeriodMeasure() {
//  current = millis() - old;
//  old = millis();
//}
//
// /*
//  * This function checks if the pleat is fully loaded
//  * The main idea is that the pleat material provides more resistance/load to dc motor when
//    all the pleat is pulled up and nothing more to pull up. The DC motor speed decreases under
//    load. Therfore, as the pleat is pulled up, the speed of the DC motors decrease (and eventually
//    goes to zero)
//  * The frequency of the disk encoder sensor (from the roller) is directly proportional to the
//    speed of the roller. In other words, the time period of the encoder sensor is inversely
//    proportional to the speed of the roller i.e. as the roller slows down the timer period of the
//    encoder sensor increases
//  * This function checks the time period of the encoder signal. When the time period increases to
//    certain value, the pleating process is stopped becuase the pleat is loaded.
//  * As explained in the TimePeriodMeasure(), a seocnd micro is used to measure time period because of
//    faulty proto-board circuit. The second mirco output low when the time period increases to certain
//    value. Therefore, this function stops the pleating process whent the second micro outputs LOW
//*/
//void CheckPleatLoading() {
//  // while (current <  1.0 / 0.9 * timePeriod) { // This line should be used when second micro is not used
//  while (digitalRead(38) == HIGH) { // Second microcontroller condition testing (NOT ideal)
//    Serial.println(current);        // No load value should be around 100
//  }
//  gripper.stopPleating();
//}
