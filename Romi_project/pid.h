#ifndef _PID_h
#define _PID_h
#include <stdint.h>

/*Here, the definition of the PID class begins. This is indicated by the keyword: "class"
This is a general description of the data and functions that the class contains. 
To use a class, we must make a specific instance of the class by declaring it into the same way we declare a variable. 
For example, to create a version of the PID class, in our main file we might write:

PID LeftWheelPID;
PID RightWheelPID;

This will create two instances of the PID class; one for the left wheel and one for the right wheel. 
Each class will have a full copy of all the variables and functions defined for that particular class.
*/ 

class PID
{
  public:

    PID(float P, float I, float D);                 // This is the class constructor. It is called whenever we create an instance of the PID class 
    void setGains(float P, float I, float D );      // This function updates the values of the gains
    void reset();                                   // This function resets any stored values used by the integral or derative terms
    float update(float demand, float measurement);  // This function calculates the PID control signal. It should be called in a loop
    void printComponents();                         // This function prints the individual components of the control signal and can be used for debugging
    void setMax(float  newMax);                     // This function sets the maximum output the controller can ask for
    void setDebug(bool state);                      // This function sets the debug flag;
    
    float error = 0.0;
    float error_deri = 0.0;
    float integral_error = 0.0;   //For storing the integral of the error
    

  private:

    //Control gains
    float Kp; //Proportional
    float Ki; //Integral
    float Kd; //Derivative

    //We can use this to limit the output to a certain value
    float max_output; 

    //Output components
    //These are used for debugging purposes
    float Kp_output; 
    float Ki_output;
    float Kd_output;
    float output_signal;

    //Values to store between updates().
    float last_demand;      //For storing the previous input
    float last_measurement; //For storing the last measurement
    float last_error;       //For calculating the derivative term

    //float error = 0.0;
    //float error_deri = 0.0;
    //float integral_error = 0.0;   //For storing the integral of the error

    long last_millis;       //To track elapsed_time
    bool debug;             //This flag controls whether we print the contributions of each component when update is called
    
};

/*
 * Class constructor
 * This runs whenever we create an instance of the class
 */
 PID::PID(float P, float I, float D)
{
  //Store the gains
  setGains(P, I, D);
  
  // Initialise key variables.
  Kp_output     = 0;
  Ki_output     = 0;
  Kd_output     = 0;
  output_signal = 0;

  max_output        = 255;
  last_demand       = 0;
  last_measurement  = 0;
  last_error        = 0;

  error = 0;
  error_deri = 0;
  integral_error    = 0;
  debug             = false;
  last_millis       = millis();
  
}

void PID::printComponents() {
  Serial.print(Kp_output);
  Serial.print(",");
  Serial.print(Kd_output);
  Serial.print(",");
  Serial.print(Ki_output);
  Serial.print(",");
  Serial.print(output_signal);
  Serial.print("\n");
}

/*
 * This function sets the gains of the PID controller
 */
void PID::setGains(float P, float I, float D) {
  Kp = P;
  Ki = I;
  Kd = D;
}


float PID::update(float demand, float measurement) {
  //Calculate how much time (in milliseconds) has passed since the last update call
  long time_now = millis();
  int time_delta = time_now - last_millis;
  last_millis = time_now;

  /*
   * ================================
   * Your code goes implementation of a PID controller should go here
   * ================================
   */

  //This represents the error term
  // Decide what your error signal is (demand vs measurement)
  //float error;
  error = demand - measurement;
  //Serial.println(error);   
  
  //This represents the error derivative
  // Calculate the change in your error between update()
  //float error_deri;
  error_deri = (error-last_error)/time_delta;

  // This represents the error integral.
  // Integrate error over time.
  integral_error = integral_error + error;

  //Attenuate above error components by gain values.
  Kp_output = Kp * error;
  Ki_output =  Ki * integral_error;
  Kd_output = Kd * error_deri;

/*
  Serial.print(Kp_output);
  Serial.print(",");
  Serial.print(Ki_output);
  Serial.print(",");
  Serial.println(Kd_output);
*/

  output_signal = Kp_output + Ki_output + Kd_output;

  /*
   * ===========================
   * Code below this point should not need to be changed
   * But of course, feel free to improve / experiment :)
   */

   
  //Update persistent variables.
  last_demand = demand;
  last_measurement = measurement;
  last_error = error;

  
  // Catching max in positive sign.
  if (output_signal > max_output) 
  {
    output_signal = max_output;
  } 
  // Catching max in negative sign
  if (output_signal < -max_output) 
  {
    output_signal = -max_output;
  }
  
  
  //Print debugging information if required
  if (debug) {
    
    Serial.print(error);
    Serial.print(",");
    Serial.print(error_deri);
    Serial.print(",");
    Serial.print(integral_error);
    Serial.print(",");
    
    printComponents();
    Serial.println();
  }
  
  return output_signal;
}

void PID::setMax(float newMax)
{
  if (newMax > 0) {
    max_output = newMax;
  } else {
    Serial.println("Max output must be positive");
  }
}

void PID::setDebug(bool state) {
  debug = state;
}

void PID::reset() {
  
  last_error = 0;
  integral_error = 0;
  last_millis = millis();
  
}

#endif
