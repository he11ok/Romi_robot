#include "variables.h" //all variables in this head file

//for the PID controller

//gain for the left wheel PID
float Kp_left = 1.5;
float Ki_left = 0.05;
//float Ki_left = 0.01;
float Kd_left = -0.02;

//gain for the right wheel PID
float Kp_right = 1.5;
float Ki_right = 0.05;
//float Ki_right = 0.01;
float Kd_right = -0.02;

//gain for heading PID
float Kp_heading = 0.6;
float Ki_heading = 0.06;
//float Ki_heading = 0.1;
float Kd_heading = -0.02;

//gain for angular PID
float Kp_angular = 1;
float Ki_angular = 0.01;
float Kd_angular = -0.02;

//gain for location PID
//float Kp_location = 0.5*0.1;
float Kp_location = 0.2*0.1;
float Ki_location = 0.0002*0.1;
float Kd_location = -0.02*0.1;

//gain for speed_lock PID
float Kp_SPlock = 0.8;
//float Ki_SPlock = 0.001;
//float Ki_SPlock = 0.1;
float Ki_SPlock = 0.08;
float Kd_SPlock = -0.8;


PID left_PID(Kp_left, Ki_left, Kd_left); // controller for left wheel
PID right_PID(Kp_right, Ki_right, Kd_right); // controller for left wheel
PID heading_PID(Kp_heading, Ki_heading, Kd_heading); // heading control
PID angular_PID(Kp_angular, Ki_angular, Kd_angular); //angular control
PID location_PID(Kp_location, Ki_location, Kd_location);//location control
PID SPlock_PID(Kp_SPlock, Ki_SPlock, Kd_SPlock);//speed lock to make left velocity equals to the right velocity

//LineSensor
#define LINE_LEFT_PIN A4 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN A2 //Pin for the right line sensor

LineSensor line_left(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
LineSensor line_centre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
LineSensor line_right(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

//Use kinematics
Kinematics Kine;


void use_PID_speed(float demand_velocity_L, float demand_velocity_R)
{
	//float demand_velocity_L, demand_velocity_R = 800;
	//    output_signal <----PID-- demand, measurement
  left_PID.setMax(2000);
  right_PID.setMax(2000);
  float output_L = left_PID.update(demand_velocity_L, count_velocity_L);
  float output_R = right_PID.update(demand_velocity_R, count_velocity_R);

  if(brake)
  {
      left_PID.reset();
      right_PID.reset();
      L_power = 0;
      R_power = 0;
      output_L = 0;
      output_R = 0;
  }
 
    L_motor_in = output_L/24;
    R_motor_in = output_R/24;
//    Serial.print("Motor power input:");
//    Serial.print(L_motor_in);
//    Serial.print(",");
//    Serial.print(R_motor_in);
//    Serial.println();
}


void use_PID_SPlock()
{
  SPlock_PID.setMax(10);
  float abs_SP_L = abs(count_velocity_L);
  float abs_SP_R = abs(count_velocity_R);
  float output_SPlock = SPlock_PID.update(abs_SP_L,abs_SP_R);
  float SPlock_error = SPlock_PID.error;
  float SPlock_safe = 15;
  //SPlock_PID.setDebug(1);
  //Serial.println(output_SPlock);
  //Serial.println(SPlock_PID.error);
  
  if(abs(SPlock_error) < SPlock_safe)
  {
  if(L_motor_in >= 0)
    {
      L_motor_in = L_motor_in - 0.5*output_SPlock;
    }
    else
    {
      L_motor_in = L_motor_in + 0.5*output_SPlock;
    }

    if(R_power >= 0)
    {
      R_motor_in = R_motor_in + 0.5*output_SPlock;
    }
    else
    {
      R_motor_in = R_motor_in - 0.5*output_SPlock;
    }
  }

  if(brake == 1)
  {
    L_power = 0;
    R_power = 0;
    L_motor_in = 0;
    R_motor_in = 0;
    heading_PID.reset();
    left_PID.reset();
    right_PID.reset();
    SPlock_PID.reset();
  }
}


void use_PID_heading()
{
  //heading_PID.setDebug(1);
	//float output_heading = heading_PID.update(0, Measure);
  heading_PID.setMax(2);
  float output_heading = heading_PID.update(0, online_confidence);
  float heading_power = output_heading * wheel_power;//wheel function require *wheel power

  L_power = heading_power;
  R_power = -heading_power;
}


void use_PID_angular(float target_theta, float theta_now)
{
  angular_PID.setMax(0.8*PI);
  float output_angular = angular_PID.update(target_theta, theta_now);
  float angular_power = output_angular * orientation_power / PI; //wheel function require *wheel power

  if(abs(angular_power)< 12.5)
  {
    if(angular_power < 0)
    {
      angular_power = -12.5;
    }
    else
    {
      angular_power = 12.5;
    }
    
  }
  L_power = angular_power;
  R_power = -angular_power;
//  Serial.print(target_theta);
//  Serial.print(",");
//  Serial.print(theta_now);
//  Serial.print(",");  
//  Serial.println(angular_power);

}


void use_PID_location(float target_location, float location_now)
{
  location_PID.setMax(speed_2);
  float output_location = location_PID.update(target_location, location_now);

  if(output_location<15)
  {
    output_location = 15;
  }

  L_power = output_location;
  R_power = output_location;
//    Serial.print("speed (count/s):");
  //  Serial.print(L_power);
  //  Serial.print(",");
  //  Serial.print(R_power);
  //  Serial.println();
//    Serial.print("demand_distance:");
//    Serial.println(demand_distance);
}


void use_Kine()
{
    Kine.update_kine(count_e0_diff, count_e1_diff);

    //consider if require the average of several values
    x_coordinate = Kine.x_globle;
    y_coordinate = Kine.y_globle;
    theta_coordinate = Kine.theta_globle;

//    Serial.print(x_coordinate);
//    Serial.print(",");
//    Serial.print(y_coordinate);
//    Serial.print(",");
//    Serial.println(theta_coordinate);
}

float convert(float content)
{
  if(content < 0)
  {
    content = 0.0;
  }

  return content;
}


float wipe_nan(float content)
{
  if(isnan(content))
  {
    content = 0.0;  
  } 
  return content;
}


void online_conf()
{
  Cali_left = line_left.readCalibrated();
  Cali_centre = line_centre.readCalibrated();
  Cali_right = line_right.readCalibrated();
  
  Cali_sum = convert(Cali_left) + convert(Cali_centre) + convert(Cali_right);

  likeli_left = convert(Cali_left) / Cali_sum;
  likeli_centre = convert(Cali_centre) / Cali_sum;
  likeli_right = convert(Cali_right) / Cali_sum;

  likeli_left = wipe_nan(likeli_left);
  likeli_centre = wipe_nan(likeli_centre);
  likeli_right = wipe_nan(likeli_right);
  
  Measure = likeli_left - likeli_right;

  measure[sample] = Measure;  
  for(int i = 0; i <= sample-1; i++)
  {
    measure[i] = measure[i+1];
  }
  measure[sample] = 0;

  for(int i = 0; i <= sample-1; i++)
  {
    Measure_sum += measure[i];
  }

  online_confidence = Measure_sum / sample;
  Measure_sum = 0.0;
  //Serial.println(online_confidence);

}
