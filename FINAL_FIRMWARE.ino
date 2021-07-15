/*  @file FINAL_FIRMWARE 
 *  Description: Control code for the VIMEE Gripper, this code 
 *               reads sensor data and performs basic feedback control.
 *  
 *  Note: This code is intended to be used with protoboard.
 *        Define new pins for the PCB, if the code does not work, 
 *        comment out TOF sensor initialization code.
 *  
 *  Please see the project status report and PCB assembly instructions for the updates that 
 *  need to be done.
 * 
 *  Included libraries:
 *  - Gripper.h
 *  - LinearMotor.h
 *  - LoadCell.h
 *  - Pins.h
 *  - RollerMotor.h
 */

/* include libraries */
#include "Gripper.h"
#include "LinearMotor.h"
#include <Wire.h>
#include "Adafruit_VL6180X.h"

/* status constants */
#define TOF_THRESHOLD 50
#define NUM_SENSORS 3
#define TOF1_CHANNEL 0
#define TOF2_CHANNEL 1
#define TOF3_CHANNEL 2
#define HEIGHT0 0
#define HEIGHT1 50
#define HEIGHT2 100
#define HEIGHT3 150
#define TCAADDR 0x70 /* define I2C address for multiplexer */

/* create three new ToF sensor objects */
Adafruit_VL6180X ToF1 = Adafruit_VL6180X();
Adafruit_VL6180X ToF2 = Adafruit_VL6180X();
Adafruit_VL6180X ToF3 = Adafruit_VL6180X();

/* create new gripper and LMotor objects */ 
Gripper gripper;
LinearMotor LMotor;

/* variable declarations */
int sensorReadings[NUM_SENSORS]; 
int currentIndex = HEIGHT0; 
byte serialData;
int force = 0;
int PleatingDir = 0; /* 0 or 1 */
int MotorSpeed = 100;
double testForce;
int currentHeight;
int newHeight;
int pleatSlip;

volatile unsigned long current;
volatile unsigned long diff;
volatile unsigned long old;
volatile bool check = LOW;
int timePeriod = 100; /* in ms */

String readString;
char c;
int n;
double l;

volatile int endStop1Tripped = 0;
volatile int endStop2Tripped = 0;
long duration;
float distance;

void setup() {
  Serial.begin(9600);
  Wire.begin(); /* enable I2C protocol */
  analogReference(EXTERNAL);

  /* input from second microcontroller??? what second microcon.? */
  pinMode(38, INPUT);
  
  /* initialize endstop pins */
  pinMode(END_STOP_PIN_1, INPUT);
  pinMode(END_STOP_PIN_2, INPUT);

  /* initialize ultrasonic sensor pins */ 
  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT); 
   
  // Only one interrupt is used for one end-stop
  attachInterrupt(digitalPinToInterrupt(END_STOP_PIN_1), ISR_ES1, CHANGE);

  // Add second interrupt for the second end-stop if needed (ISR Already coded)
  //attachInterrupt(digitalPinToInterrupt(END_STOP_PIN_2), ISR_ES2, CHANGE);

  // Interruppt for the encoder disk sensor that is already on the gripper
  // TO DO: Add interrupt for the second encoder disk
  pinMode(FRONT_ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FRONT_ENCODER_PIN), TimePeriodMeasure, RISING);

  Serial.println(F("##############################"));            
  Serial.println(F("Starting Initialization"));
  Serial.println(F("##############################")); 

  /* initializing 1st ToF sensor */
  tcaselect(TOF1_CHANNEL); 
  if (!ToF1.begin()) {
    Serial.print(F("ToF1 detected?\t"));
    Serial.print(F("No"));
    Serial.println();
  }
  else {
    Serial.print(F("ToF1 detected?\t"));
    Serial.print(F("Yes"));
    Serial.println();
  }

  /* initializing 2nd ToF sensor */
  tcaselect(TOF2_CHANNEL);
  if (!ToF2.begin()) {
    Serial.print(F("ToF2 detected?\t"));
    Serial.print(F("No"));
    Serial.println();
  }
  else {
    Serial.print(F("ToF2 detected?\t"));
    Serial.print(F("Yes"));
    Serial.println();

  }

  /* initializing 3rd ToF sensor */
  tcaselect(TOF3_CHANNEL);
  if (!ToF3.begin()) {
    Serial.print(F("ToF3 detected?\t"));
    Serial.print(F("No"));
    Serial.println();
  }
  else {
    Serial.print(F("ToF3 detected?\t"));
    Serial.print(F("Yes"));
    Serial.println();
  } 

  Serial.println();
  Serial.println(F("##############################"));            
  Serial.println(F("Initialization Finished"));
  Serial.println(F("##############################"));             
  Serial.println();
  Serial.println();
  
  /* command definitions */
  Serial.println("Ready to Accept Commands......");
  Serial.println("SEE COMMENTS IN THE CODE FOR MORE DETAILS....");
  
  /* unused commands */
  //Serial.println("Press a for automatic mode.....");
  //Serial.println("l for loading pleat up");
  //Serial.println("s for start Pleating movement"); // need to change
  //Serial.println("q for quitting Automatic Mode"):

  /* DC roller motor commands */
  Serial.println("\n    DC ROLLER MOTOR COMMANDS");
    // Enter z100 for 100% duty cycle
    // Enter z50 for 50% duty cycle and so on
    Serial.println("z: left motor clockwise, eg. z100");
    // Enter x100 for 100% duty cycle
    // Enter x50 for 50% duty cycle and so on
    Serial.println("x: left motor cClockwise, eg. x100");
    // Enter ,100 (i.e. [comma]100) for 100% duty cycle
    // Enter ,50 for 50% duty cycle and so on
    Serial.println(".: right motor clockwise, eg. ,100");
    // Enter /100 for 100% duty cycle
    // Enter /50 for 50% duty cycle and so on
    Serial.println("/: right motor cClockwise eg. /100");


  /* linear motor commands */
  Serial.println("\n    LINEAR MOTOR COMMANDS");
    // Enter [5000 for moving 5000 steps forward
    // Enter [1000 for moving 1000 steps forward and so on
    Serial.println("[: linear Motor forward(Close) [5000");
    // Enter ]5000 for moving 5000 steps backward
    // Enter ]1000 for moving 1000 steps backward and so on
    Serial.println("]: linear Motor backward(Open) ]5000");


  /* sensor output */
  Serial.println("\n    SENSOR OUTPUT COMMANDS");
  Serial.println("L: print loadcell readings continously");
  Serial.println("y: print loadcell voltage readings once");
  Serial.println("V: print TOF sensors status continously");
  Serial.println("T: Encoder 1 period output continously");
  Serial.println("u: Ultrasonic sensor output continously");

  /* useful functions */
  Serial.println("\n    USEFUL FUNCTION COMMANDS");
    // I: Run motor 1 clockwise and motor2 Counter-clockwise (pushing pleat out) (100% duty cycle)
    // i: Run motor 1 counter-clockwise and motor2 clockwise (pulling pleat up) (100% duty cycle)
    Serial.println("I and i: Rotate rollers in opposite direction (pushing/pulling pleat)");
    // O: Run both motors counter-clockwise (100% duty cycle)
    // o: Run both motors clockwise (100% duty cycle)
    Serial.println("O and o: Rotate rollers in same direction (moving pleat)");
    // e.g. c2.1 means the mirco closes the claw until 2.1 Kg of force is applied
    Serial.println("c: Close Claw with pre-determined force in Kg eg. c2");  
    // Open claws with moving the linear motor pre-determined steps
    // Check LinearMotor.h code for setting the number of steps
    Serial.println("h: Open Claws");
    // Loadcell Data print mode with Linear motor control
    // The loadcell data is printed in the serial monitor/logger
    // commands in terms of steps can be given to move the linear motor in the closing direction
    // Eg. enter 500 to move 500 steps in closing direction (Not [500 nor ]500 like above, just the number)
    Serial.println("L: Loadcell data print mode with linear motor control");
    // Automatic mode: Loads the pleat, stop loading the pleat when all the pleat is inside,
    //                 Moves the pleat for given amount of time (check code for setting time)
    Serial.println("P: automatc Pleat Load, stop, move Pleat");
    // Stop Roller Motors
    Serial.println("S: Stop Roller Motors");
}

void loop() {
  // printing endstops if they are tripped
  if (endStop1Tripped == 1)
    Serial.println("End stop 1 tripped.....");
  if (endStop2Tripped == 1)
    Serial.println("End stop 2 tripped..... ");

  if (Serial.available() > 0) {
    serialData = Serial.read();
  }
  
  /* if-else commands explained in set-up function*/ 
  if (serialData == 'c') {
    LMotor.CloseClaws(force);
  }
  else if (serialData == 'l') {
    gripper.LoadPleat(MotorSpeed);
  }
  else if (serialData == 's') {
    gripper.StartPleating(MotorSpeed, PleatingDir);
  }
  else if (serialData == 'h') {
    LMotor.OpenClaws();
  }
  else if (serialData == 'z') {
    Serial.println("Enter PWM for left motor: clockwise");
    while (!Serial.available());
    while (Serial.available()) {
      c = Serial.read(); // gets one byte from serial buffer
      readString += c; //makes the string readString
      delay(10); //slow looping to allow buffer to fill with next character
    }
    n = readString.toInt();
    // Serial.println(n);
    gripper.leftMotorControl(n, 0);
    readString = "";
  }
  else if (serialData == 'x') {
    // getting the duty cycle from serial input
    while (!Serial.available());
    while (Serial.available()) {
      c = Serial.read(); // gets one byte from serial buffer
      readString += c; // makes the string readString
      delay(10); // slow looping to allow buffer to fill with next character
    }
    n = readString.toInt();
    // Serial.println(n);
    // using the duty cycle to set the motor speed
    gripper.leftMotorControl(n, 1);
    readString = "";
  }
  else if (serialData == ',') {
    // getting the duty cycle from serial input
    while (!Serial.available());
    while (Serial.available()) {
      c = Serial.read(); // gets one byte from serial buffer
      readString += c; // makes the string readString
      delay(10); // slow looping to allow buffer to fill with next character
    }
    n = readString.toInt();
    // Serial.println(n);
    // using the duty cycle to set the motor speed
    gripper.rightMotorControl(n, 0);
    readString = "";
  }
  else if (serialData == '/') {
    // getting the duty cycle from serial input
    while (!Serial.available());
    while (Serial.available()) {
      c = Serial.read();  //gets one byte from serial buffer
      readString += c; //makes the string readString
      delay(10);  //slow looping to allow buffer to fill with next character
    }
    n = readString.toInt();
    // Serial.println(n);
    // using the duty cycle to set the motor speed
    gripper.rightMotorControl(n, 1);
    readString = "";
  }
  else if (serialData == '[') {
    // getting the number of steps from serial input for linear motor
    while (!Serial.available());
    while (Serial.available()) {
      c = Serial.read(); // gets one byte from serial buffer
      readString += c; // makes the string readString
      delay(10); // slow looping to allow buffer to fill with next character
    }
    n = readString.toInt();
    // Serial.println(n);
    // moving the input number of steps
    LMotor.MoveSteps(n, 0);
    readString = "";
  }
  else if (serialData == ']') {
    // getting the number of steps from serial input for linear motor
    while (!Serial.available());
    while (Serial.available()) {
      c = Serial.read(); // gets one byte from serial buffer
      readString += c; // makes the string readString
      delay(10); // slow looping to allow buffer to fill with next character
    }
    n = readString.toInt();
    // Serial.println(n);
    // moving the input number of steps
    LMotor.MoveSteps(n, 1);
    readString = "";
  }
  else if (serialData == 'L') {
    while (1) {
      // Serial.println(LMotor.GetLoadCellvolt());
      Serial.println(Filter_2());
      // Serial.println(loadcell.GetCal());
      // Serial.println(LMotor.GetLoadCellReading());
      if (Serial.available()) {
        while (Serial.available()) {
          c = Serial.read(); // gets one byte from serial buffer
          readString += c; // makes the string readString
          delay(100); // slow looping to allow buffer to fill with next character
        }
      }
      n = readString.toInt();
      // Serial.println(n);
      LMotor.MoveSteps(n, 0);
      readString = "";
      delay(1000);
    }
  }
  else if (serialData == 'T') {
    while (1) {
      if (current > 0) {
        Serial.println(current);
        delay(100);
      }
    }
  }
  else if (serialData == 'P') {
    LMotor.CloseClaws(0.1);
    gripper.LoadPleat(100);
    delay(1000);
    CheckPleatLoading();
    delay(2000);
    LMotor.MoveSteps(350, 1);
    gripper.rightMotorControl(100, 1);
    gripper.leftMotorControl(100, 1);
    readSensors();
    for(int counter=NUM_SENSORS-2; counter>=0; counter--){
      Serial.print(" Sensor ");
      Serial.print(counter);
      Serial.print(" ");
      Serial.println(sensorReadings[counter]);
    }    
  }
  else if (serialData == 'y') {
    while (1) {
      // Serial.println(LMotor.GetLoadCellvolt());
      Serial.println(Filter_y());
      if (Serial.available()) {
        while (Serial.available()) {
          l = Serial.read(); // gets one byte from serial buffer
          readString += l; // makes the string readString
          delay(100); // slow looping to allow buffer to fill with next character
        }
      } 
    }
  } 
  else if (serialData == 'O') {
    gripper.rightMotorControl(100, 1);
    gripper.leftMotorControl(100, 1);
  }
  else if (serialData == 'o') {
    gripper.rightMotorControl(100, 0);
    gripper.leftMotorControl(100, 0);
  }
  else if (serialData == 'I') {
    gripper.rightMotorControl(100, 0);
    gripper.leftMotorControl(100, 1);
  }
  else if (serialData == 'i') {
    gripper.rightMotorControl(100, 1);
    gripper.leftMotorControl(100, 0);
  }
  else if (serialData == 'S') {
    //LMotor.CloseClaws(1);
    //gripper.LoadPleat(100);
    //delay(100);
    //CheckPleatLoading();
    gripper.rightMotorControl(0, 1);
    gripper.leftMotorControl(0, 0);
  }
  /*
  else if (serialData == 'm') {
    testForce = 0.1;
    while (testForce < 2) {
      //LMotor.OpenClaws();
      LMotor.MoveSteps(1500, 1);
      Serial.print("Testing with ");
      Serial.print(testForce);
      Serial.println(" Kg: Press any key to continue");
      while (!Serial.available());

      LMotor.CloseClaws(testForce);

      gripper.rightMotorControl(100, 0);
      gripper.leftMotorControl(100, 1);
      delay(2000);
      gripper.rightMotorControl(100, 0);
      gripper.leftMotorControl(100, 0);
      testForce = testForce + 0.1;
    }
  }
  */
  else if (serialData == 'V'){
    readSensors();
    while(1) {
      readSensors();
      for(int i = 0; i < NUM_SENSORS; i++) {
        Serial.print("ToF ");
        Serial.print(i+1);
        Serial.print(": ");
        Serial.print(sensorReadings[i]);
        Serial.println();
      }
      delay(300);
      Serial.println();
    }
  }

  else if (serialData == 'v') {
    currentHeight = getPleatHeight();
    while(1) {
      newHeight = getPleatHeight();
      pleatSlip = checkPleatSlipping(currentHeight, newHeight);      
      if (pleatSlip == 1) {
        Serial.print("Pleat has slipped - please reload.");
        Serial.end();
      } 
      Serial.print("Pleat height: ");
      Serial.print(newHeight);
      Serial.println();  
      currentHeight = newHeight;
    }
  }  
  else if(serialData == 'u'){
    
    while(1){
      digitalWrite(TRIG_PIN, LOW); // Clears the trigPin
      delayMicroseconds(2);  
      digitalWrite(TRIG_PIN, HIGH); // Sets the trigPin on HIGH state for 10 micro seconds
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
      duration = pulseIn(ECHO_PIN, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds 
      //distance= 15.25-duration*0.034/2; // Calculating the distance
      distance= duration*0.034/2;
      Serial.println(distance);
      delay(100);
    }
  }
  serialData = 0;
}


/**************************************************/
/*****ToF Sensors*******/
/* Function: Enables multiplexer to toggle between its eight channels
 *           communicate with any connected devices.
 */
void tcaselect(uint8_t bus) {
  if (bus > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

/* Function: Reads one given ToF sensor.*/
int readSensor(int sensorNum) {
  tcaselect(sensorNum);
  if (sensorNum = TOF1_CHANNEL) return ToF1.readRange();
  else if (sensorNum = TOF2_CHANNEL) return ToF2.readRange();
  else if (sensorNum = TOF3_CHANNEL) return ToF3.readRange();
  else return 0;
}

/* Function: Reads all ToF sensors and stores values to the 
 *           'SensorReadings' array; 1 if sensor is tripped, 
 *           0 otherwise.
 */
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (readSensor(i) <= TOF_THRESHOLD) {
      sensorReadings[i] = 1;
    }
    else {
      sensorReadings[i] = 0;
    }
  }
}

/* Function: Returns pleat height in terms of index of last 
 *           sensor triggered by the pleat.
 */
int getPleatHeight() {
  int counter = 0;
  currentIndex = counter;
  readSensors(); // get updated readings for each sensor
  
  while ((sensorReadings[counter] == 1) && (counter <= 2)) {
    currentIndex = counter + 1;
    counter++;
    readSensors();
  }

  if (currentIndex == 1) return HEIGHT1;
  else if (currentIndex == 2) return HEIGHT2;
  else if (currentIndex == 3) return HEIGHT3;
  else return HEIGHT0; // pleat has yet to trigger any sensor
}

/* Function: Checks if pleat is slipping; returns 1 if slipping
 *           and 0 otherwise.
 */
int checkPleatSlipping(int currentHeight, int newHeight) { 
  if (newHeight < currentHeight) return 1;
  else return 0;
}


/**************************************************/
/*****Filters*******/
#define FILTER_N 100
float Filter() {
  int i, j;
  float filter_temp, filter_sum = 0;
  float filter_buf[FILTER_N];
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = LMotor.GetLoadCellReading();
    delay(1);
  }
  // 采样值从小到大排列（冒泡法）
  for(j = 0; j < FILTER_N - 1; j++) {
    for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf[i] > filter_buf[i + 1]) {
        filter_temp = filter_buf[i];
        filter_buf[i] = filter_buf[i + 1];
        filter_buf[i + 1] = filter_temp;
      }
    }
  }
  // 去除最大最小极值后求平均
  for(i = 1; i < FILTER_N - 1; i++) filter_sum += filter_buf[i];
  return filter_sum / (FILTER_N - 2);
}

#define FILTER_N 12
float filter_buf[FILTER_N + 1];
float Filter_2() {
  int i;
  float filter_sum = 0;
  filter_buf[FILTER_N] = LMotor.GetLoadCellReading();
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位仍掉
    filter_sum += filter_buf[i];
  }
  return (float)(filter_sum / FILTER_N);
}

#define FILTER_N 100
float Filter_y() {
  int i, j;
  float filter_temp, filter_sum = 0;
  float filter_buf[FILTER_N];
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = LMotor.GetLoadCellvolt();
    delay(1);
  }
  // 采样值从小到大排列（冒泡法）
  for(j = 0; j < FILTER_N - 1; j++) {
    for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf[i] > filter_buf[i + 1]) {
        filter_temp = filter_buf[i];
        filter_buf[i] = filter_buf[i + 1];
        filter_buf[i + 1] = filter_temp;
      }
    }
  }
  // 去除最大最小极值后求平均
  for(i = 1; i < FILTER_N - 1; i++) filter_sum += filter_buf[i];
  return filter_sum / (FILTER_N - 2);
}


/**************************************************/
/*****Encoders*******/
/*
 * ISR Function for encoder 1:
 * This function measure the time between the rising edges of the square wave signal 
   of the encoder sensor (Time period of the signal)
 * e.g. If the encoder signal is 10 Hz square wave, current will be 1/10Hz = 100ms
   (i.e. the time period of signal)
 
 NOTE: A second microcontroller was used to do the this function beacause of a problem
 with the protoboard so THIS FUNCTION IS NOT USED IN THE CURRENT CODE. Refer to code for
 the secondary microcontroller.
 TO DO: Use this function with main microcontroller instead of secondary micrcontoller 
        along with the CheckPleatLoading()
        (Eliminate the need to the second microcontroller
 */
void TimePeriodMeasure() {
  current = millis() - old;
  old = millis();
}

 /*
  * This function checks if the pleat is fully loaded
  * The main idea is that the pleat material provides more resistance/load to dc motor when 
    all the pleat is pulled up and nothing more to pull up. The DC motor speed decreases under
    load. Therfore, as the pleat is pulled up, the speed of the DC motors decrease (and eventually
    goes to zero)
  * The frequency of the disk encoder sensor (from the roller) is directly proportional to the 
    speed of the roller. In other words, the time period of the encoder sensor is inversely
    proportional to the speed of the roller i.e. as the roller slows down the timer period of the
    encoder sensor increases
  * This function checks the time period of the encoder signal. When the time period increases to
    certain value, the pleating process is stopped becuase the pleat is loaded.
  * As explained in the TimePeriodMeasure(), a seocnd micro is used to measure time period because of
    faulty proto-board circuit. The second mirco output low when the time period increases to certain
    value. Therefore, this function stops the pleating process whent the second micro outputs LOW
*/
void CheckPleatLoading() {
  // while (current <  1.0 / 0.9 * timePeriod) { // This line should be used when second micro is not used
  while (digitalRead(38) == HIGH) { // Second microcontroller condition testing (NOT ideal)
    Serial.println(current);        // No load value should be around 100
  }
  gripper.StopPleating();
}
// This ISR is for endstop sensor 1
// When there is falling edge, the sensor got triggered
// When there is rising edge, the sensor got cleared
void ISR_ES1() {
  // When the sensor is triggered, disable further movement in that direction
  if (digitalRead(END_STOP_PIN_1) == 0) { 
    endStop1Tripped = 1;
    // Stop Motors
    LMotor.Disable();
    LMotor.DisableDir1();
    gripper.StopPleating();
    CheckPleatLoading();
  }
  else { //When sensor is cleared, enable the linear motor movement
    endStop1Tripped = 0;
    LMotor.Enable();
    LMotor.EnableDir1();
  }
}

// This ISR is for endstop sensor 1
// When there is falling edge, the sensor got triggered
// When there is rising edge, the sensor got cleared
// NOTE: THIS ISR is not used in the code right now
void ISR_ES2() {
  // When the sensor is triggered, disable further movement in that direction
  // However, the sensor is not at good position so nothing is disabled
  if (digitalRead(END_STOP_PIN_2) == 0) { 
    endStop2Tripped = 1;
    // Stop Motors
    //LMotor.Disable();
    //LMotor.DisableDir2();
    //gripper.StopPleating();
  }
  else { //When sensor is cleared, enable the linear motor movement
    endStop2Tripped = 0;
    LMotor.Enable();
    LMotor.EnableDir2();
  }
}
