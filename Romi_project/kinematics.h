#ifndef _Kinematics_h
#define _Kinematics_h

//You may want to use some/all of these variables
const float WHEEL_DIAMETER    = 70;
const float WHEEL_RADIUS      = 35;
//const float CAR_LENGTH        = 134; //140
//const float CAR_LENGTH        = 134;
//const float CAR_LENGTH        = 138; //140
//const float CAR_LENGTH        = 142;
const float CAR_LENGTH        = 142;
const float WHEEL_SEPERATION  = CAR_LENGTH*0.5; //140*0.5
const float GEAR_RATIO        = 1/120;
//const float COUNTS_PER_SHAFT_REVOLUTION = ??;
const float COUNTS_PER_WHEEL_REVOLUTION = 1440;
const float COUNTS_L_REVOLUTION = 1438;
const float COUNTS_R_REVOLUTION = 1436;
const float COUNTS_PER_MM              = 6.6481;
const float MM_PER_COUNT                = 0.1527;
const float MM_L_COUNT                      = 0.1529;
const float MM_R_COUNT                     = 0.1531;
//const float micros_to_seconds               =  0.01; //(elapsed_time / 1000000)



class Kinematics
{
  public:
    
    Kinematics();   // Constructor, required.

    void update_kine(float count_e0_diff, float count_e1_diff);
    void debug();
    void reset();
    double x_globle, y_globle;
    //float sin_theta_globle, cos_theta_globle;
    double theta_globle;
    
  private:
    long last_millis;  
    double x_local, y_local;
    //float x_globle, y_globle;
    double x_change, y_change;
    double theta_change;
    double angular_velocity_L, angular_velocity_R;
    double velocity_contribution, velocity_difference;
    double count_contribution, count_difference;
    
};


// Required constructor.  Initialise variables.
Kinematics::Kinematics() 
{
  x_local = 0.0;
  y_local = 0.0;

  x_globle = 0.0;
  y_globle = 0.0;
  theta_globle = 0.0;

  x_change = 0.0;
  y_change = 0.0;
  theta_change = 0.0;


  last_millis = millis();
}

void Kinematics::update_kine(float count_e0_diff, float count_e1_diff) 
{
  count_contribution = (count_e0_diff + count_e1_diff)*MM_PER_COUNT;
  count_difference = (count_e0_diff - count_e1_diff)*MM_PER_COUNT;
  //count_contribution = count_e0_diff * MM_L_COUNT+ count_e1_diff * MM_R_COUNT;
  //count_difference = count_e0_diff * MM_L_COUNT - count_e1_diff * MM_R_COUNT;

  x_local = (count_contribution/2);
  y_local = 0;
  //attention here CAR_LENGTH is 2 times than the distance from cntre to the wheel
  theta_change = (count_difference / (CAR_LENGTH));

  x_change = x_local*cos(theta_globle);
  y_change = x_local*sin(theta_globle);

  x_globle += x_change;
  y_globle += y_change;

  //update the globle variables for theta
  theta_globle += theta_change;
  if(theta_globle > 2*PI)
  {
    theta_globle = theta_globle - 2*PI;
  }

  if(theta_globle < 0)
  {
    theta_globle = theta_globle + 2*PI;
  }

  return;
}

/*
void Kinematics::update_kine(float count_velocity_L, float count_velocity_R, float elapsed_time) 
{
  angular_velocity_L = (count_velocity_L * MM_PER_COUNT) /WHEEL_RADIUS;
  angular_velocity_R = (count_velocity_R * MM_PER_COUNT) /WHEEL_RADIUS;
  //pay attention to the velocity is count/s
  velocity_contribution = (angular_velocity_L + angular_velocity_R) *WHEEL_RADIUS;
  velocity_difference = (angular_velocity_L - angular_velocity_R) *WHEEL_RADIUS;

  x_local = (velocity_contribution/2) * (elapsed_time / 1000000) ;
  y_local = 0;
  //attention here CAR_LENGTH is 2 times than the distance from cntre to the wheel
  theta_change = (velocity_difference / CAR_LENGTH) * (elapsed_time / 1000000);
  //sin_theta_change = velocity_contribution/CAR_LENGTH;
  //cos_theta_change = sqrt(1 - sin_theta_change^2);

  x_change = x_local*cos(theta_globle);
  y_change = x_local*sin(theta_globle);

  x_globle += x_change;
  y_globle += y_change;

  //update the globle variables for theta
  theta_globle += theta_change;
  if(theta_globle > 2*PI)
  {
    theta_globle = theta_globle - 2*PI;
  }

  if(theta_globle < 0)
  {
    theta_globle = theta_globle + 2*PI;
  }

  return;
}

*/

void Kinematics::debug()
{
    Serial.print(x_globle);
    Serial.print(",");
    Serial.print(y_globle);
    Serial.print(",");
    Serial.println(theta_globle);
}

void Kinematics::reset()
{
  x_globle = 0.0;
  y_globle = 0.0;
  theta_globle = 0.0;
}

#endif
