#include <EnableInterrupt.h>
#include <Servo.h>
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
double throttle_ref;
double RPM_L;
Servo MotorL;
Servo MotorR;
Servo MotorB;
Servo Motor_yaw;
 char a;
 String b;
 int count;
 char arr=' ';
 double data_0;
 double data_1;
 double data_2;
 double data_3;
 double roll, pitch, yaw, prev_yaw;
 double sensor_x, sensor_y, sensor_z;
 int data_count;
 int accelero_count;
/*
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
*/

void setup()
{
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial2.begin(57600);
  MotorL.attach(4);
MotorR.attach(5);
MotorB.attach(6);
Motor_yaw.attach(8);
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  throttle_ref=0;
  RPM_L=0;
  count=0;
roll=0;
pitch=0;
yaw=0;
data_count=0;
data_0=0;
data_1=0;
data_2=0;
data_3=0;
accelero_count=0;
sensor_x=0;
sensor_y=0;
sensor_z=0;
prev_yaw=0;

}
void loop()
{
// rc_read_values();
  get_accelero_values();
  if (millis()%50==0)
{
  /*
Serial.println(roll);
Serial.println(pitch);
Serial.println(yaw);
*/
Serial2.println("sensor_x");
Serial2.println(sensor_x);
Serial2.println("sensor_y");
Serial2.println(sensor_y);
Serial2.println("sensor_z");
Serial2.println(sensor_z);
Serial2.println("roll");
Serial2.println(roll);
Serial2.println("pitch");
Serial2.println(pitch);
Serial2.println("yaw");
Serial2.println(yaw);

//Serial2.println("Thrust_motors");
//Serial2.println(rc_values[RC_CH3]);

Serial.println(pitch);
}

  /*
throttle_ref=map(rc_values[RC_CH3], 1050, 1850, 0, 255);
RPM_L = map(throttle_ref, 0, 255, 1000, 2000);
MotorL.writeMicroseconds(RPM_L);
MotorR.writeMicroseconds(RPM_L);
MotorB.writeMicroseconds(1000);
Motor_yaw.writeMicroseconds(1000);
Serial.println(RPM_L);
*/
//imu();
}
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
/*
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
*/
void get_accelero_values()
{
if (Serial1.available())
{
  a=Serial1.read();
  if (count==0)
  {
   if (a=='$')
   {
    count=1;
   } 
  }
  if (count == 1)
  {
    
  if (a==',')
  {
    data_count++;
    if (data_count==3)
    {
      data_0= b.toDouble();
    }
    if (data_count==4)
    {
    data_1= b.toDouble();
  //Serial.println(data);
    }
    if (data_count==5)
    {
      data_2= b.toDouble();
    }
    if (data_count==6)
    {
      data_3= b.toDouble();
    }
   b="";
   // Serial.println(data_count);
  }
  if (a!=',' && data_count > 1 )
  {
    b+=a;
  }
   if (a=='*')    // Check Sum 
   {
    if (data_count==5)
    {
      roll= data_0;
      pitch= data_1;
      yaw= data_2;
    }
    if (data_count==6)
    {
      if (accelero_count<1)
      {
 //  Serial.println(data_1);
   sensor_x= data_1;
   sensor_y= data_2;
   sensor_z= data_3;
      }
   accelero_count++;
   if (accelero_count>2)
   {
    accelero_count=0;
   }
    }
    count=0;
    b="";
    data_count=0;
    data_1=0;
    data_2=0;
    data_3=0;
   }
}
//Serial.println(roll);
//Serial.println(test);
}
//Serial.println(b);
//Serial.println(roll);
}

