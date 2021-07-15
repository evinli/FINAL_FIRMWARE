// Load cell data Acquisition 
//
//
// TO DO:
// Add load cell calibration function

#include "Pins.h"
// The class loadcell initialize the loadcell and provide 
// load cell readings

class LoadCell {
    double voltage;
    double kg;
    double kg_sensor;
    
    // Calibration Constant for the Load Cell
    const double GRADIENT = 0.0271;
    double OFFSET = 103.72;

  public:
    LoadCell() {
      pinMode(LOAD_CELL_OUT, INPUT);
    }

// The function results the force applied on the loadcell in Kg
// NEED TO CHANGE THIS FUNCTION WHEN USING PCB. THe PCB has an 
// external voltage reference

    double GetForce() {
      voltage = averageAnalog(LOAD_CELL_OUT)*(2500/1023); 
      kg_sensor = ((voltage - OFFSET)/GRADIENT)/1000;
      // adding correction curve
      kg = 2.1139*kg_sensor+0.5178;
     
      return kg;
    }

// Function for load cell calibration: Need to Improve this function
// Get raw data from the loadcell right now.
    double GetCal() {
      voltage = averageAnalog(LOAD_CELL_OUT)*(1.0*2500/1023);
      
      return voltage;
    }
    
//Averaging function for the load cell
    int averageAnalog(int pin) {
      int v = 0;
      //for (int i = 0; i < 4; i++) v += analogRead(pin);
      
      return analogRead(pin);
    }

};
