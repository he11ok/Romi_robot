#define black_threshold 300 
#define sample 10

#define wheel_power 30
#define orientation_power 20
#define speed_1 18
#define speed_2 25
#define low_speed 15

//for the encoders
volatile float last_count_e0, last_count_e1;
volatile double count_velocity_L, count_velocity_R;
volatile unsigned long last_timestamp1, last_timestamp2;
volatile unsigned long elapsed_time1, elapsed_time2;
volatile unsigned long time_now1, time_now2;
volatile float velocity_L, velocity_R;
volatile float count_e0_diff, count_e1_diff;

float Cali_left, Cali_centre, Cali_right = 0;
float Cali_sum, Cali_diff, Cali_diff_sum, Cali_diff_avg = 0; 

float Measure, Measure_sum = 0.0;
float online_confidence = 0.0;
float likeli_left, likeli_centre, likeli_right = 0.0;

volatile float measure[sample+1];
bool off_white = 0;
bool brake = 0;

int behavior = 1;
int goback_behavior = 1;
bool online = 0;
bool go_home = 0;
bool orientation_correct = 0;
bool location_correct = 0;
int offline_time = 0;
float L_power, R_power = 0.0;
float L_motor_in, R_motor_in = 0.0;

int last_case = 0;

volatile double x_coordinate, y_coordinate, theta_coordinate = 0.0;
double demand_orientation = PI; //must in [0,2*PI]
double demand_distance = 0.0;
