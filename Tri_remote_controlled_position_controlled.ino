#include "Ublox.h"
#include <math.h>
#include <Servo.h>
#include <PID_v1.h>
#include <EnableInterrupt.h>
#include <Adafruit_BMP085.h>
//////////////////////////////// GPS definitions ////////////////////

#define GPS_BAUD 9600
#define N_doubleS 10

//////////////////////////////// Recieving code from RC transmiter and reciever ////////////////////////////////
#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4 

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2
#define RC_CH4_INPUT  A3

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];
/////////////////////////////////////////////// Class for error filteration ///////////////////////////

class filter{
   public:
  double input_variable;
  double output_variable;
  double count;
  double error;
  filter()
  {
  input_variable = 0;
  output_variable= 0;
  count=0;
  error=0;
  }
  void get_smooth_value(double a)
  {
   input_variable = a;
   error = abs(output_variable-input_variable);
   if ( input_variable > output_variable)
   {
    output_variable = output_variable + error/20;
   }
   if ( input_variable < output_variable)
   {
    output_variable = output_variable - error/20;
   }

  }

};
///////////////////////// Kalman Filter ////////////////////////

class kalman_filter
{
  
  public:
    double kalman_q;
    double kalman_r;
    double kalman_x;
    double kalman_p;
    double kalman_k;
  kalman_filter()
  {
    kalman_q=0.001;
    kalman_r=0.01;
    kalman_x=0;
    kalman_p=0.85;
    kalman_k=1.8;
  }
 void kalman(double data)
{
  kalman_p=kalman_p+kalman_q;
  
  kalman_k=kalman_p/(kalman_p+kalman_r);
  kalman_x=kalman_x+kalman_k*(data-kalman_x);
  kalman_p=(1-kalman_k)*kalman_p;
}
  
};

///////////////////////////////// Module Objects ////////////////////

Ublox M8_Gps;
Adafruit_BMP085 bmp;

//////////////////////////////////// definition of filters ///////////////////////

filter smooth_throttle;
filter smooth_roll_ref;
filter smooth_baro;

///////////////////// GPS definitions and Initializations ///////////////
//kalman_filter baro_smooth;
kalman_filter k_latitude;
kalman_filter k_longitude;
kalman_filter k_distance;
double lat0,lon0,lat1,lon1,lat3,lon3,B_deg0,B_deg;
double lat0a,lon0a,lat1a,lon1a,lat3a,lon3a;
double M8_latitude;
double M8_longitude;
double latitude_filter;
double longitude_filter;
double latitude_kalman;
double longitude_kalman;
int count,count_previous,count_gps,count_kalman;
uint16_t M8_speed;
unsigned long previous_millis=0;
unsigned long interval=15000;
// Altitude - Latitude - Longitude - N Satellites
double gpsArray[N_doubleS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/////////////////////////////////////////////// IMU Variables /////////////////////////////////////////

double roll = 0; // IMU Roll Value (Feedback)
double pitch = 0; // IMU Pitch value (Feedback)
double yaw = 0; // IMU Yaw (Feedback)
float height = 0; // Lidar Height Value (Feedback)
float lati; // GPS Latitude Value (Feedback)
float longi; // GPS Longitude Value (Feedback)
float roll_prev = 0; // Record of Previous Value to prevent Garbage
float pitch_prev = 0;
float yaw_prev = 0;
float height_prev = 0;
int test_count = 0;
int timestart; // Time record for response time measurement
int timestop; // Time record for response time measurement
int kill = 1; // Kill=1 => maunual
float hover_rpm = 6000;//5840;

//////////////////////////////////////////////////////////////////

double initial_height;
double roll_ref_PID;
double pitch_ref_PID;
double drift_in_x_to_maintain;
double drift_in_y_to_maintain;
double drift_in_x;
double drift_in_y;
double  roll_ref;
double pitch_ref;
double throttle_ref;
double yaw_ref;
double desired_roll;
double desired_pitch;
double prev_throttle_ref;
double desired_yaw;
double pitch_error;
double yaw_error;
double roll_error;
double throttle_error;
double  throt_L;
double throt_R;
double throt_B;
double thrust_servo;
double roll_L;
double roll_R;
double initial_pitch;
double initial_roll;
double RPM_L;
double RPM_R;
double RPM_B;
double RPM_yaw;
double thrust_for_pitch_L;
double thrust_for_pitch_R;
double thrust_for_pitch_B;
double thrust_for_roll_L;
double thrust_for_roll_R;
double throttle_for_yaw;
double first_pitch;
double first_roll;
double first_yaw;
double pitch_upper;
double pitch_lower;
double roll_upper;
double roll_lower;
int initial_values;
int Sonar=A7;
double Sonar_value;
Servo MotorL;
Servo MotorR;
Servo MotorB;
Servo Motor_yaw;
PID pitch_L_PID(&pitch, &thrust_for_pitch_L, &desired_pitch, 5, 4, 1, DIRECT);
PID pitch_R_PID(&pitch, &thrust_for_pitch_R, &desired_pitch, 5, 4, 1, DIRECT);
PID pitch_B_PID(&pitch, &thrust_for_pitch_B, &desired_pitch, 5, 4, 1, REVERSE);
PID roll_R_PID(&roll, &thrust_for_roll_R, &desired_roll, 3.5, 4, 1, REVERSE);
PID roll_L_PID(&roll, &thrust_for_roll_L, &desired_roll, 3.5, 4, 1, DIRECT);
PID yaw_servo(&yaw, &thrust_servo, &desired_yaw, 2, 1.5, 1, DIRECT);
//PID yaw_servo(&yaw, &thrust_servo, &desired_yaw, 2, 1.5 , 0.7, DIRECT);
PID maintain_x_PID(&drift_in_x, &roll_ref_PID, &drift_in_x_to_maintain, 1, 1, 0.5, DIRECT);
PID maintain_y_PID(&drift_in_y, &pitch_ref_PID, &drift_in_y_to_maintain, 1, 1, 0.5, REVERSE);

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Serial1.begin(115200);
Serial2.begin(GPS_BAUD);
MotorL.attach(4);
MotorR.attach(5);
MotorB.attach(6);
Motor_yaw.attach(8);
initial_height=0;
drift_in_x_to_maintain=0;
drift_in_y_to_maintain=0;
drift_in_x=0;
drift_in_y=0;
thrust_servo= 110;
RPM_yaw=0;
RPM_B=0;
RPM_L=0;
RPM_R=0;
roll_ref=0;
pitch_ref=0;
throttle_ref=0;
yaw_ref=0;
prev_throttle_ref=0;
throt_L=0;    // intial equivalent to 1500
throt_R=0;    // initial equivalent to 1500
throt_B=0;
initial_pitch=0;
initial_roll=0;
pitch_error=0;
yaw_error=0;
roll_error=0;
initial_values=0;
thrust_for_pitch_L=0;
thrust_for_pitch_R=0;
thrust_for_pitch_B=0;
thrust_for_roll_L=0;
thrust_for_roll_R=0;
throttle_for_yaw=0;
throttle_error=0;
first_pitch=0;
first_roll=0;
first_yaw=0;
pitch_upper=0;
pitch_lower=0;
roll_upper=0;
roll_lower=0;
lat0=0;
lon0=0;
lat1=0;
lon1=0;
lat3=0;
lon3=0;
lat0a=0;
lon0a=0;
lat1a=0;
lon1a=0;
lat3a=0;
lon3a=0;
count=0;
count_previous=0;
count_gps=0;
count_kalman=0;
B_deg0=0;
B_deg=0;
M8_speed=0;
M8_latitude=0;
M8_longitude=0;
latitude_filter=0;
longitude_filter=0;
latitude_kalman=0;
longitude_kalman=0;
roll_ref_PID=0;
pitch_ref_PID=0;

///////////////////////// PID MODE ///////////////////////////////

pitch_R_PID.SetMode(AUTOMATIC);
pitch_B_PID.SetMode(AUTOMATIC);
pitch_L_PID.SetMode(AUTOMATIC);
roll_R_PID.SetMode(AUTOMATIC);
roll_L_PID.SetMode(AUTOMATIC);
yaw_servo.SetMode(AUTOMATIC);
maintain_x_PID.SetMode(AUTOMATIC);
maintain_y_PID.SetMode(AUTOMATIC);
pitch_R_PID.SetSampleTime(30);
pitch_B_PID.SetSampleTime(30);
pitch_L_PID.SetSampleTime(30);
roll_R_PID.SetSampleTime(30);
roll_L_PID.SetSampleTime(30);
yaw_servo.SetSampleTime(30);
maintain_x_PID.SetSampleTime(100);
maintain_y_PID.SetSampleTime(100);

///////////////////////////////////////////////////////////////

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  
////////////////////////////////////////////////////////////////

maintain_x_PID.SetOutputLimits(-15,15);
maintain_y_PID.SetOutputLimits(-15,15);
/*
pitch_L_PID.SetOutputLimits(-10,245);
pitch_R_PID.SetOutputLimits(-10,245);
pitch_B_PID.SetOutputLimits(-10,245);
roll_R_PID.SetOutputLimits(-10,245);
roll_L_PID.SetOutputLimits(-10,245);
yaw_servo.Compute();
*/
MotorL.writeMicroseconds(1000);
MotorR.writeMicroseconds(1000);
MotorB.writeMicroseconds(1000);
Motor_yaw.writeMicroseconds(1500);

if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
delay(1000);
}


void loop() {
imu();
rc_read_values();
smooth_throttle.get_smooth_value(rc_values[RC_CH3]);
/////////////////// Correcting Yaw /////////////////////
yaw=yaw-20;
////////////////// Reading GPS Values ////////////////////////////////

GPS_calculations();
get_baro_values();
drift_in_x=gpsArray[8];
drift_in_y=gpsArray[9];

/////////////////////////// Zero Values /////////////////////////////
//This segment of the code initializes motors only when the throttle stick is down
// which is represented by smooth_throttle.output_variable being less than 1100
if (rc_values[RC_CH3]<1100)
{
MotorL.writeMicroseconds(1000);
MotorR.writeMicroseconds(1000);
MotorB.writeMicroseconds(1000);
Motor_yaw.writeMicroseconds(1500);
thrust_servo= 110;
thrust_for_pitch_L=0;
thrust_for_pitch_R=0;
thrust_for_pitch_B=0;
thrust_for_roll_L=0;
thrust_for_roll_R=0;
RPM_L=0;
RPM_R=0;
RPM_B=0;
RPM_yaw=0;
throttle_ref=0;
first_pitch=pitch;
first_roll=roll;
first_yaw=yaw;
desired_pitch=pitch;
desired_roll=roll;
desired_yaw=yaw;
initial_height=smooth_baro.output_variable;
}
else
{
// This segment of the code designates the attitude at the ground level as the intial reference attitude when the 
// throttle stick is very low
if (smooth_throttle.output_variable > 1100 && smooth_throttle.output_variable< 1250 )
{
desired_pitch = pitch;
desired_roll = roll;
desired_yaw = yaw;
}

//////////////////// Transmission Data ///////////////////////////
// This segment of the code limits the values of roll pitch and yaw from the initial attitudes at the ground level

pitch_upper=first_pitch + 10;
pitch_lower=first_pitch - 10;
roll_upper=first_roll + 10;
roll_lower=first_roll - 10;
//roll_ref= map(rc_values[RC_CH1], 1088 , 1918, roll_lower , roll_upper);
//pitch_ref=map(rc_values[RC_CH2], 1076 , 1882, pitch_upper , pitch_lower);
roll_ref= map(roll_ref_PID, -15 , 15, roll_lower , roll_upper);
pitch_ref=map(pitch_ref_PID, -15 , 15, pitch_lower , pitch_upper);
yaw_ref=map(rc_values[RC_CH4], 1060 , 1880, -20 , 20);
throttle_ref=map(smooth_throttle.output_variable, 1050, 1850, 0, 255);
if (pitch>(pitch_upper + 30))
{
  pitch=(pitch_upper + 30);
}
if (pitch< (pitch_lower - 30))
{
  pitch=(pitch_lower - 30);
}
if (roll>roll_upper)
{
  roll=roll_upper;
}
if (roll<roll_lower)
{
  roll=roll_lower;
}

////////////////////////////// Error Calculation /////////////////////////////////////

pitch_error=abs(pitch_ref - first_pitch);
//yaw_error= gpsArray[6] - yaw ;
roll_error=roll_error=abs(roll_ref - first_roll);
//throttle_error=abs(abs(throttle_ref)- abs(prev_throttle_ref));

///////////////////////////////////////////////////////////////////////////////////////
if (smooth_throttle.output_variable > 1300)
{
if (roll_error > 1  )
{
 desired_roll= desired_roll + (roll_error/20);
}
if  (pitch_error > 1)
{
 desired_pitch= desired_pitch + (pitch_error/20);
}
 if (yaw_ref < -1)
  {
    desired_yaw= desired_yaw + (yaw_ref/10);
  }
  if (yaw_ref > 1)
  {
    desired_yaw = desired_yaw + (yaw_ref/10);
  }
  
pitch_L_PID.Compute();
pitch_R_PID.Compute();
pitch_B_PID.Compute();
roll_R_PID.Compute();
roll_L_PID.Compute();

yaw_servo.Compute();

if (smooth_baro.output_variable > initial_height + 2)
{
maintain_x_PID.Compute();
maintain_y_PID.Compute();
}
}

/////////////////////////// PID computation /////////////////////////////////

/*
pitch_L_PID.Compute();
pitch_R_PID.Compute();
pitch_B_PID.Compute();
roll_R_PID.Compute();
roll_L_PID.Compute();
yaw_servo.Compute();
*/

////////////////////// Throttle Values ////////////////////////

throt_L = ((0.8*throttle_ref) + (.3*((thrust_for_pitch_L + thrust_for_roll_L)/2)));
throt_R = ((0.8*throttle_ref) + (.3*((thrust_for_pitch_R + thrust_for_roll_R)/2)));
throt_B= (0.8*throttle_ref) + (.3*(thrust_for_pitch_B/2));

///////////////////// RPM values ////////////////////////

RPM_L = map(throt_L, 0, 255, 1000, 2000);
RPM_R = map(throt_R, 0, 255, 1000, 2000);
RPM_B = map(throt_B, 0, 255, 1000, 2000);
RPM_yaw = map(thrust_servo, 0, 255, 1300, 1800);

/////////////////////////// Motor RPM ///////////////////////////

MotorL.writeMicroseconds(1000);
MotorR.writeMicroseconds(1000);
MotorB.writeMicroseconds(1000);
Motor_yaw.writeMicroseconds(1500);

/////////////////////////////////////////////////////////////////
}
/*
Serial.print("pitch: ");
Serial.println(pitch);
Serial.print("error: ");
Serial.println(pitch_error);
Serial.print("ref: ");
Serial.println(pitch_ref);
Serial.print("prev: ");
Serial.println(desired_pitch);
Serial.println("roll");
Serial.println(roll);
Serial.println("throttle_reference");
Serial.println(throttle_ref);
Serial.println("roll reference");
Serial.println(desired_roll);
Serial.println("channel value");
Serial.println(rc_values[RC_CH3]);
Serial.println("servo throttle");
Serial.println(thrust_servo);
Serial.println("right throttle roll");
Serial.println(thrust_for_roll_R);

Serial.println("reference roll");
Serial.println(roll_ref);
Serial.println("roll");
Serial.println(roll);
Serial.println("previous roll");
Serial.println(desired_roll);
Serial.println("roll error");
Serial.println(roll_error);
Serial.println("right throttle roll");
Serial.println(thrust_for_roll_R);
Serial.println("left throttle roll");
Serial.println(thrust_for_roll_L);
Serial.println("snar value");
Serial.println(Sonar_value);
Serial.println("yaw_error");
Serial.println(yaw_error);
Serial.println(smooth_throttle.output_variable);
Serial.println(rc_values[RC_CH3]);
Serial.println("RPM_L");
Serial.println(RPM_L);
Serial.println("RPM_R");
Serial.println(RPM_R);
Serial.println("RPM_B");
Serial.println(RPM_B);
Serial.println("Reference Roll");
Serial.println(roll_ref);
Serial.println("Roll error");
Serial.println(roll_error);
Serial.println("desired roll");
Serial.println(desired_roll);
Serial.println("Roll");
Serial.println(roll);
Serial.println("channel 1");
Serial.println(rc_values[RC_CH1]);
Serial.println("channel 2");
Serial.println(rc_values[RC_CH2]);
Serial.println("channel 3");
Serial.println(rc_values[RC_CH3]);
Serial.println("channel 4");
Serial.println(rc_values[RC_CH4]);

Serial.println("desired yaw");
Serial.println(desired_yaw);
Serial.println("reference yaw");
Serial.println(yaw_ref);
Serial.println("yaw");
Serial.println(yaw);
Serial.println("yaw RPM");
Serial.println(RPM_yaw);
Serial.println("yaw error");
Serial.println(yaw_error);

Serial.println("Reference Roll");
Serial.println(roll_ref);
Serial.println("Roll error");
Serial.println(roll_error);
Serial.println("desired roll");
Serial.println(desired_roll);
Serial.println("Roll");
Serial.println(roll);
Serial.println("channel 1");
Serial.println(rc_values[RC_CH1]);
Serial.println("channel 2");
Serial.println(rc_values[RC_CH2]);
Serial.println("channel 3");
Serial.println(rc_values[RC_CH3]);
Serial.println("channel 4");
Serial.println(rc_values[RC_CH4]);
Serial.println("Reference Roll");
Serial.println(roll_ref);
Serial.println("Roll error");
Serial.println(roll_error);
Serial.println("desired roll");
Serial.println(desired_roll);
Serial.println("Roll");
Serial.println(roll);
Serial.println("desired yaw");
Serial.println(desired_yaw);
Serial.println("reference yaw");
Serial.println(yaw_ref);
Serial.println("yaw");
Serial.println(yaw);
Serial.println("yaw RPM");
Serial.println(RPM_yaw);
Serial.println("yaw error");
Serial.println(yaw_error);
Serial.print("pitch: ");
Serial.println(pitch);
Serial.print("error: ");
Serial.println(pitch_error);
Serial.print("ref: ");
Serial.println(pitch_ref);
Serial.print("desired pitch ");
Serial.println(desired_pitch);
Serial.println("Reference Roll");
Serial.println(roll_ref);
Serial.println("Roll error");
Serial.println(roll_error);
Serial.println("desired roll");
Serial.println(desired_roll);
Serial.println("Roll");
Serial.println(roll);

Serial.println("RPM_L");
Serial.println(RPM_L);
Serial.println("RPM_R");
Serial.println(RPM_R);
Serial.println("RPM_B");
Serial.println(RPM_B);
Serial.println("yaw");
Serial.println(yaw);
*/
/*
for(byte i = 0; i < N_doubleS; i++) 
{
Serial.print(gpsArray[i],6); Serial.print(" ");
}
Serial.println("");
Serial.println(M8_Gps.speed/100);
//Serial.println(latitude_filter,5);
//Serial.println(longitude_filter,5);
Serial.println(latitude_kalman,6);
Serial.println(longitude_kalman,6);

Serial.println("pitch_ref");
Serial.println(pitch_ref);
Serial.println("roll_ref");
Serial.println(roll_ref);

Serial.println(baro_smooth.kalman_x);
*/
Serial.println(drift_in_x);
//Serial.println(drift_in_y);
//Serial.println(yaw);
}

//////////////////////////////// IMU Data ////////////////////////////////////////////////
void imu() {
    //int datalength = 20; //112-116 (5 Registers)
    int i = -1;
    char ch = 0;
    int count = 0;
    uint16_t temp;
    uint8_t in[20] = {NULL}; // 's','n','p',PT,Address,Data,Checksum[1],Checksum[0]
    //////////////////////////////Parsing Binary Data /////////////////////////////////////////
    while(1)
    {
      if(Serial1.available()>0)
      {
          i = Serial1.read();
          if(i == -1);
              //Serial.println("No Data");
          else
          {
              //Serial.println("SUCCESS");
              ch = i;
              //Serial.println(ch);
              if(in[0] == 's' && in[1] == 'n' && in[2] =='p')
              {
                  count = count+1;
                  in[count] = ch;
              }
              else if(ch == 's')
              {
                  count = 0;
                  in[count] = 's';
              }
              else if(ch == 'n')
              {
                  count = 1;
                  in[count] = 'n';
              }
              else if(ch == 'p')
              {
                  count = 2;
                  in[count] = 'p';
              }
          }
            if(count == 17)
            {
                count = 0;
                if(in[4] == 112)
                {
                    temp = in[5] * 256 + in[6];
                    roll = ((int16_t) temp) / 91.02222;
                    //Serial.print("Roll :");
                    //Serial.print(roll);
                    //Serial.print(" ");
                    temp = in[7] * 256 + in[8];
                    pitch = ((int16_t) temp) / 91.02222 ;
                    //Serial.print("pitch :");
                    //Serial.print(pitch);
                    //Serial.print(" ");
                    temp = in[9] * 256 + in[10];
                    yaw = ((int16_t) temp) / 91.02222;
                    //Serial.print("yaw :");
                    //Serial.println(yaw);
                    break; // Break if Roll,Pitch,Yaw Acquired
                    }
                    for(i=0 ; i<20 ; i++)
                    in[i] = NULL;
                }
        }
    }
}
////////////////////////////// Transmission Data /////////////////////////////////////
void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }

/////////////////// Get Initial Point ///////////////////////////////

 void get_initial(double x1, double y1)
{
  
  if(lat1>33.6129 && count==0)
  {
    if(count_previous==0)
    {
      previous_millis=millis();
      count_previous++;
    }
    unsigned long current_millis=millis();
    if((unsigned long)(current_millis-previous_millis)>=interval)
    {
      lat0=x1;
      lon0=y1;
      count++;
      previous_millis=millis();
    }
  }
  gpsArray[4]=lat0;
  gpsArray[5]=lon0;
  
}
//////////////////////Get Bearing Final////////////////////////

void get_bearing_final(double x3,double y3)
{
  
//lat3=x3;
//lon3=y3;
lat3a=x3*M_PI/180;
lon3a=y3*M_PI/180;

double delL=lon3a-lon1a;
/*
double X=cos(lat3a)*sin(delL);
double Y=cos(lat1a)*sin(lat3a)-sin(lat1a)*cos(lat3a)*cos(delL);
double B_rad=atan2(X,Y);
*/

double X=log(tan(lat3a/2.0+M_PI/4.0)/tan(lat1a/2.0+M_PI/4.0));
if (abs(delL) > M_PI)
{
    if (delL > 0.0)
       delL = -(2.0 * M_PI - delL);
    else
       delL = (2.0 * M_PI + delL);
}

double B_rad=atan2(delL,X);

B_deg=B_rad*180/M_PI;
if(B_deg>180)
{
  B_deg=B_deg-360.0;
}
gpsArray[6]=B_deg;

}

///////////////////Get Bearing Initial///////////////////

void get_bearing_initial()
{

double delL0=lon1a-lon0a;
double X0=log(tan(lat1a/2.0+M_PI/4.0)/tan(lat0a/2.0+M_PI/4.0));
if (abs(delL0) > M_PI)
{
    if (delL0 > 0.0)
       delL0 = -(2.0 * M_PI - delL0);
    else
       delL0 = (2.0 * M_PI + delL0);
}

double B_rad0=atan2(delL0,X0);
B_deg0=B_rad0*180/M_PI;
if(B_deg0>180)
{
  B_deg0=B_deg0-360.0;
}

//gpsArray[7]=B_deg0;
  
}


/////////////////////////// Get Drift ///////////////////////////////

void get_drift(double current_yaw)
{
  
  double dlat= lat1a-lat0a;
  double dlon= lon1a-lon0a;
  double a=(sin(dlat/2)*sin(dlat/2))+(cos(lat0a)*cos(lat1a)*sin(dlon/2)*sin(dlon/2));
  double b= 2*atan2(sqrt(a),sqrt(1-a));
  double distance=6371000*b;
  k_distance.kalman(distance);
  double error=B_deg0-current_yaw+90;
  double error_rad=error*M_PI/180;
  double x=k_distance.kalman_x*sin(error_rad);
  double y=k_distance.kalman_x*cos(error_rad);
  gpsArray[8]=x;
  gpsArray[9]=y;
}

void GPS_calculations()
{
  while(Serial2.available())
{
char c = Serial2.read();
if (M8_Gps.encode(c)) 
{
gpsArray[0] = M8_Gps.altitude;
gpsArray[1] = M8_Gps.sats_in_use;
gpsArray[2] = M8_Gps.latitude;
gpsArray[3] = M8_Gps.longitude;
}
}

////////////////// Calculating GPS Data /////////////////////////////

M8_latitude=M8_Gps.latitude*100000;
M8_longitude=M8_Gps.longitude*100000;
if(count_kalman==0)
  {
    k_latitude.kalman_x=M8_latitude;
    k_longitude.kalman_x=M8_longitude;
    count_kalman++;
  }
k_latitude.kalman(M8_latitude);
k_longitude.kalman(M8_longitude);
latitude_kalman=k_latitude.kalman_x/100000;
longitude_kalman=k_longitude.kalman_x/100000;

/*lat1=latitude_filter;
lon1=longitude_filter;*/
lat1=latitude_kalman;
lon1=longitude_kalman;
lat1a=lat1*M_PI/180;
lon1a=lon1*M_PI/180;
lat0a=lat0*M_PI/180;
lon0a=lon0*M_PI/180;

/*get_initial(latitude_filter,longitude_filter);*/
get_initial(latitude_kalman,longitude_kalman);
get_bearing_initial();
//get_bearing_final();
get_drift(yaw);
}
void get_baro_values()
{
 smooth_baro.get_smooth_value(bmp.readAltitude(101500));
// smooth_height_baro=baro.output_variable;
}

