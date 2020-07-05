#ifndef _Line_follow_h
#define _Line_follow_h

//Number of readings to take for calibration
const int NUM_CALIBRATIONS = 50;


class LineSensor
{
  public:

    // Required function, class Constructor: 
    // Saves the pin passed in as argument and sets to input
    LineSensor(int line_pin) {
      pin = line_pin;
      pinMode(pin, INPUT);
    }

    // Suggested functions.
    float calibrate();       //Calibrate
    float readRaw();         //Return the uncalibrated value from the sensor
    float readCalibrated();  //Return the calibrated value from the sensor
    volatile int Cali_time = 0;

    // You may wish to add other functions!
    // ...
    
  private:
  
    int pin;
    float error,avg_error;
    float Rawvalue_sum;
    float calibrated_value;
    float Raw_value;
    

    /*
     * Add any variables needed for calibration here
     */
    
};


// Returns unmodified reading.
float LineSensor::readRaw()
{
  float reading = analogRead(pin);
  if(isnan(reading))
  {
    reading = 0.0;
   }
   //Serial.println(reading);
  return reading;
}

// Write this function to measure any
// systematic error in your sensor and
// set some bias values.
float LineSensor::calibrate()
{ 
  int i;
  Rawvalue_sum = 0;
  error = 0;

    for(i=0;i<NUM_CALIBRATIONS;i++)
    {
      Rawvalue_sum += readRaw();
    }

  error = Rawvalue_sum/NUM_CALIBRATIONS;

  return error;

}


// Use the above bias values to return a
// compensated ("corrected") sensor reading.
float LineSensor::readCalibrated()
{
  if(Cali_time == 0)
  {
    avg_error = calibrate();
    Cali_time = 1;
  }
  calibrated_value =0;
  calibrated_value = readRaw() - avg_error;
  //Serial.println(calibrated_value);
  return calibrated_value; 
}



#endif
