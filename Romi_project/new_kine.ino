#include "math.h"
#include "Car_go.h"
#include "encoders.h"
#include "pid.h"
#include "lineSensors.h"
#include "kinematics.h"
#include "default_setup.h"


void setup() 
{
	//setup the interrupt
	setupEncoder0();
	setupEncoder1();
	setupMotor();

	Serial.begin(9600);

	delay(5000);
	Serial.println("***RESET***");
	last_timestamp1 = micros();
	last_timestamp2 = micros();
	last_count_e0 = count_e0;
	last_count_e1 = count_e1;

}


void loop() 
{
  online_conf();
  time_now1 = micros();
  elapsed_time1 = time_now1 - last_timestamp1;
  time_now2 = micros();
  elapsed_time2 = time_now2 - last_timestamp2;

  if (elapsed_time1 > 10000)
  {
    last_timestamp1 = micros();

    count_e0_diff = count_e0 - last_count_e0;
    count_e1_diff = count_e1 - last_count_e1;

    //update the time and the count
    last_count_e0 = count_e0;
    last_count_e1 = count_e1;


    //velocity unit as count/s
    count_velocity_L = 1000000* count_e0_diff /elapsed_time1 ;
    count_velocity_R = 1000000* count_e1_diff /elapsed_time1 ;

    //velocity convert to the mm/s
    //1 count = 0.15mm
    velocity_L = 0.1527 * count_velocity_L;
    velocity_R = 0.1527 * count_velocity_R;

    //The main behavior codes
    use_Kine();//before the action use kinematics
    //Kine.debug();
    //action2();  //action2 for testing the go home function
    action();
    //debug_this();
    
  }


  //use 2nd micros periodly update the speed demand in PID
  if (elapsed_time2 > 5000)
  {
    last_timestamp2 = micros();

    //use_Kine();

    use_PID_speed(L_power*24,R_power*24);

    use_PID_SPlock();

    car_motor(L_motor_in,R_motor_in);
  }
 
}


void debug_this()
{
  /*
  Serial.println();
  Serial.print("Sensor Reading:    ");
  Serial.print(Cali_left);
  Serial.print(",");
  Serial.print(Cali_centre);
  Serial.print(",");
  Serial.print(Cali_right);
  Serial.println();

  Serial.print("Measure:    ");
  Serial.print(Measure);
  Serial.println();

  Serial.print("online_confidence:    ");
  Serial.print(online_confidence);
  Serial.println();

  //Serial.print("speed (count/s):    ");
  Serial.print(count_velocity_L);
  Serial.print(",");
  Serial.print(count_velocity_R);
  Serial.println();


  Serial.print("X Y theta:    ");
  Serial.print(x_coordinate);
  Serial.print(",");
  Serial.print(y_coordinate);
  Serial.print(",");
  Serial.print(theta_coordinate);
  Serial.println();
 */
}


void action()
{
    switch (behavior)
  {
  case 1:
    if(line_left.Cali_time&&line_centre.Cali_time&&line_right.Cali_time)
    {
      beep(1);
      behavior ++;
    }
    break;

  case 2:
     if(online)
    {
      beep(2);
      behavior ++;
    }
    else
    {
    find_line(); 
    }
    break;

  case 3:
    if(online)
    {
      MBangbang_pid();
      online_check();
    }
    else
    {
      L_power = 0;
      R_power = 0;
      behavior ++;
    }
    break;

  case 4:
    L_power = 0;
    R_power = 0;
    L_motor_in = 0;
    R_motor_in = 0;
    left_PID.reset();
    right_PID.reset();
    heading_PID.reset();
    SPlock_PID.reset();
    Serial.print("finish line following task");
    beep(2);
    behavior ++;
    break;

  case 5:
    //car_move(0);
    delay(1000);
    behavior ++;

  case 6:
    Go_back();
    break;
  
  default:
    break;
  }

}


int case_1, case_2, case_3, case_4 = 0;

void action2()
{
  switch (behavior)
  {
  case 1:
  {
    //car_motor(20, 20);
    brake = 0;
    L_power = 20;
    R_power = 20;
    case_1 ++;
    if(case_1>500)
    {
      brake = 1;
      behavior ++;
    }
  }
    break;
  
  case 2:
  {
    //car_motor(20, -20);
    brake = 0;
    L_power = -25;
    R_power = 25;
    case_2 ++;
    if(case_2 > 150)//300
    {
      brake = 1;
      behavior ++;
    }
  }
    break;
    
  case 3:
  {
    //car_motor(20, 20);
    brake = 0;
    L_power = 20;
    R_power = 20;
    case_3 ++;
    if (case_3 > 200)
    {
      brake = 1;
      behavior ++;
    }
  }
    break;

  case 4:
  {
    //car_motor(0,0);
    brake = 1;
    L_power = 0;
    R_power = 0;
    case_4++;
    if(case_4>100)
    {
      behavior ++;
    }
  }
    break;

  case 5:
  {
    brake = 0;
    behavior++;
  }
    break;

  case 6:
  {
    Go_back();
  }
    break;

  default:
    break;
  }

}



void find_line()
{
  //if( (Cali_centre > black_threshold) ) //possible change to belief
  if ( (Cali_centre > black_threshold))
  {
    //car_motor(0,0);
    L_power = 0;
    R_power = 0;
    online = 1;
  }
  else
  {
    //car_motor(speed_1,speed_1);
    L_power = speed_1;
    R_power = speed_1;
  }
}


void online_check()
{
  off_white = ((Cali_left < 50)&&(Cali_centre < 50)&&(Cali_left < 50))? 1:0;
  //off_white = ((likeli_left < 0.3)&&(likeli_centre < 0.3)&&(likeli_right < 0.3))? 1:0;
  //Serial.println(off_white);
  if(off_white)
  {
    offline_time ++;
  }
  
  if(offline_time > 100 )
  {
    online = 0;
  }
}

void MBangbang_pid()
{
  bool centre_only =  ( (likeli_centre >= 0.7) || ( (online_confidence < 0.15)&&(online_confidence > -0.15) ) )? 1:0 ;
  //bool all_black = ( (Cali_left > 300)&&(Cali_right > 300)&&(Cali_centre > 300) )? 1:0 ;
  //bool centre_only = (Cali_centre > black_threshold)? 1:0 ;

  if(off_white)
  {
      //car_move(low_speed);
      L_power = low_speed;
      R_power = low_speed;
    }
    else
    {
    if(centre_only)
    {
      //car_move(speed_1);
      L_power = speed_1;
      R_power = speed_1;
      online = 1;
      offline_time = 0;
      heading_PID.reset();
    }
    else
    {
      use_PID_heading();
    }
  }
        
}


bool get_final_Kine = 1;

void Go_back()
{
  go_home = 1;

  switch (goback_behavior)
  {
  case 1:
  {
    //get the final theta of the robot when plan to go back only one time
    if(get_final_Kine)
    {
      left_PID.reset();
      right_PID.reset();
      float location_angular = atan2(x_coordinate,y_coordinate);
      demand_orientation = (PI - location_angular) + 0.5*PI;
      if(y_coordinate< 0)
      {
        demand_orientation = demand_orientation* 0.9832;
      }
      else
      {
        demand_orientation = demand_orientation* 0.9748;
      }
      //Serial.println(demand_orientation);
      //demand_orientation = (demand_orientation < theta_coordinate)? (demand_orientation + 2*PI):(demand_orientation); //turn clockwise
      get_final_Kine = 0;
    }

    use_PID_angular(demand_orientation, theta_coordinate);

    orientation_correct =  ( (Kine.theta_globle>=demand_orientation*0.99) && (Kine.theta_globle<=demand_orientation*1.01) ) ? 1:0; 
    //||( (Kine.theta_globle>=(demand_orientation*0.99-2*PI)) && (Kine.theta_globle<=(demand_orientation*1.01-2*PI)) )  )? 1:0;

    if(orientation_correct)
    {
      brake = 1;
      goback_behavior ++;
      get_final_Kine = 1;
    }

  }
  break;

  case 2:
  {
    delay(1000);
    goback_behavior ++;
  }

  case 3:
  {
    //after adjust the final orientation, get final Kine again to acquire a final_x & y
    if(get_final_Kine)
    {
      float location_distance = sqrt( sq( x_coordinate ) + sq( y_coordinate) );
      demand_distance = location_distance;
      Kine.reset();//reset the x_globle, y_globle & theta_globle
      get_final_Kine = 0;
      brake = 0;
      use_Kine();//reuse the Kine extra one time just at here!!
    }

    //use_Kine();//if this one can not use?? because the update rate? possible!
    float location_now = sqrt( sq( x_coordinate ) + sq( y_coordinate ) );
    
    use_PID_location(demand_distance,location_now);
    location_correct = ( (location_now>=demand_distance*0.9825) && (location_now<=demand_distance*0.990) )? 1:0;

    if(location_correct)
    {
      brake = 1;
      Kine.reset();
      goback_behavior ++;
    }

  }
   break;

  case 4:
  {
    beep(2);
    goback_behavior ++;
  }
    break;
  
  default:
    break;
 }

}
