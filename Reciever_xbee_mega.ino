char a;
String b;
String ref;
int count=0;
float roll=0;
float pitch=0;
float yaw=0;
float roll_bt=0;
float pitch_bt=0;
float yaw_bt=0;
float Thrust_motor=0;
float thrust_bt=0;
float Sensor_x=0;
float Sensor_y=0;
float Sensor_z=0;
float sensor_x_bt=0;
float sensor_y_bt=0;
float sensor_z_bt=0;
void setup()
{
Serial.begin(9600);
Serial1.begin(57600);
Serial2.begin(9600);
}

void loop()
{
 
recieve_values();
//right_to_bluetooth();
}
 
void right_to_bluetooth()
{
  if (millis()%50==0)
  {
  roll_bt=(roll/10)+100;
   Serial2.println(roll_bt);
   pitch_bt=pitch/10;
   Serial2.println(pitch_bt);
   yaw_bt=(yaw/10)+200;
   Serial2.println(yaw_bt); 
   sensor_x_bt=(Sensor_x/10)+300;
   Serial2.println(sensor_x_bt);
    sensor_y_bt=(Sensor_y/10)+400;
   Serial2.println(sensor_y_bt);
    sensor_z_bt=(Sensor_z/10)+500;
   Serial2.println(sensor_z_bt);
   thrust_bt= ((Thrust_motor/100)+4500);
   Serial2.println(thrust_bt);
  }
}
void recieve_values()
{
    if (Serial1.available())
  {
a=Serial1.read();
if (a=='\n')
{
  if(count==1 && ref=="roll")
  {
    ref="";
    count=0;
    roll= b.toFloat();
  // Serial.println(roll,5);
  }
  if(count==2 && ref=="pitch")
  {
    ref="";
    count=0;
    pitch= b.toFloat();
  Serial.println(pitch);
  }
  if(count==3 && ref=="yaw")
  {
    ref="";
    count=0;
    yaw= b.toFloat();
 // Serial.println(yaw,5);
  }
   if(count==4 && ref=="Thrust_motors")
  {
    ref="";
    count=0;
    Thrust_motor= b.toFloat();
 // Serial.println(Thrust_motor);
  }
  if(count==5 && ref=="sensor_x")
  {
    ref="";
    count=0;
    Sensor_x= b.toFloat();
  //Serial.println(Sensor_x);
  }
   if(count==6 && ref=="sensor_y")
  {
    ref="";
    count=0;
    Sensor_y= b.toFloat();
 // Serial.println(Sensor_y);
  }
   if(count==7 && ref=="sensor_z")
  {
    ref="";
    count=0;
    Sensor_z= b.toFloat();
  //Serial.println(Sensor_z);
  }
  if (b.lastIndexOf("roll")>0)
  {
    ref="roll";
    count=1;
  }
  if(b.lastIndexOf("pitch")>0)
  {
    ref="pitch";
    count=2;
  }
  if(b.lastIndexOf("yaw")>0)
  {
    ref="yaw";
    count=3;
  }
  if(b.lastIndexOf("Thrust_motors")>0)
  {
    ref="Thrust_motors";
    count=4;
  }
  if (b.lastIndexOf("sensor_x")>0)
  {
    ref="sensor_x";
    count=5;
  }
  if (b.lastIndexOf("sensor_y")>0)
  {
    ref="sensor_y";
    count=6;
  }
  if (b.lastIndexOf("sensor_z")>0)
  {
    ref="sensor_z";
    count=7;
  }
b="";
}
b+=a;
}
}



