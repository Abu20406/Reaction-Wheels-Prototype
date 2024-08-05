#include <Wire.h>
#include <PID_v1.h>

//Motor 1 for pitch control
int direction1 = 8;
int speed1 = 9;
//Motor 3 for roll control
int direction3 = 4;
int speed3 = 3;

int i = 0;
bool direction_3 = HIGH;
bool direction_1 = HIGH;

// Creating 16 bit variables
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 
const int MPU = 0x68;
float Acceleration_angle[2];
float Gyro_angle[3];
float Total_Angle[2];

float elapsedTime, time, timePrev;
float rad_to_deg = 180/3.141592654;

float PID_R,PID_P, pwmMotor1,pwmMotor3, errorRoll, errorPitch, previous_error_Roll, previous_error_Pitch;
//for 2 axis control I think I will add errorPitch and previous_error_Pitch and similarly
//the pid_p pid_i etc values

//PID for roll
float pid_p_roll=0;
float pid_i_roll=0;
float pid_d_roll=0;

//PID for pitch
float pid_p_pitch=0;
float pid_i_pitch=0;
float pid_d_pitch=0;

//PID constants for roll
double kp_R = 3.5;
double ki_R=1.5;
double kd_R=2;

//PID constants for pitch
double kp_P=3.3;
double ki_P=.6;
double kd_P=1.4;

float desired_angle_pitch = -1; //This is the angle in which we whant the
                         //balance to stay steady
float desired_angle_roll = -7;

//Check the desires_angle. Meausre the rest position angle and adjust it. Might not be 0 neccessarily.

void setup() {
  Serial.begin(115200);
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Configuring the Accelerometer Sensitivity
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talking to the ACCEL_CONFIG register 1C 
  Wire.write(0x00);                  //+-2g sensitivity
  Wire.endTransmission(true);
  
  // Configuring Gyro Sensitivity
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                   // 250deg/s sensitivity
  Wire.endTransmission(true);
  delay(20);

  time = millis(); //Start counting time in milliseconds

  pinMode(speed1,OUTPUT);
  pinMode(speed3,OUTPUT);
  pinMode(direction1,OUTPUT);
  pinMode(direction3,OUTPUT);
}

void loop() {

    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 

    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Reading the 6 accelerometer measurement registers from 3B to 40.
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,6,true); 
    
     Acc_rawX = Wire.read()<<8|Wire.read(); 
     Acc_rawY = Wire.read()<<8|Wire.read();
     Acc_rawZ = Wire.read()<<8|Wire.read();
     
     /*---X---*/
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    
   Wire.beginTransmission(MPU);
   Wire.write(0x43); // Reading the 6 gyroscope measurement registers from 43 to 48
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true); 
   
   Gyr_rawX = Wire.read()<<8|Wire.read(); 
   Gyr_rawY = Wire.read()<<8|Wire.read();
   Gyr_rawZ = Wire.read()<<8|Wire.read(); 
 
   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; //ROLL
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0; //PITCH
   /*---Z---*/
   Gyro_angle[2] = Gyr_rawZ/131.0; //YAW

   //ROLL
   Total_Angle[0] = 0.98 *(Total_Angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   //PITCH
   Total_Angle[1] = 0.98 *(Total_Angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

   Total_Angle[2] =  Total_Angle[2] + Gyro_angle[2] * elapsedTime;

    Serial.print("ROLL: ");
    Serial.println(Total_Angle[0]);

    Serial.print("PITCH: ");
    Serial.println(Total_Angle[1]);
    delay(100);
   
//Calculating error between current angle and desired angle
errorRoll = desired_angle_roll-Total_Angle[0]; //if the roll is lesser than -10 this is always positive.
errorPitch = desired_angle_pitch-Total_Angle[1]; //if pitch is -ve this value is always positive.

pid_p_roll = kp_R*errorRoll;
pid_p_pitch = kp_P*errorPitch;

//Integral control is used only for small errors
if(-3 <errorRoll <3) {pid_i_roll = pid_i_roll+(elapsedTime*errorRoll);  }
else {pid_i_roll = 0; }
if(-3 <errorPitch <3){pid_i_pitch = pid_i_pitch+(elapsedTime*errorPitch);  }
else {pid_i_pitch = 0; }

pid_d_roll = kd_R*((errorRoll - previous_error_Roll)/elapsedTime);
pid_d_pitch = kd_P*((errorPitch - previous_error_Pitch)/elapsedTime);

PID_R = pid_p_roll + (ki_R*pid_i_roll)+pid_d_roll;
PID_P = pid_p_pitch + (ki_P*pid_i_pitch)+pid_d_pitch;

//if the total PID is -ve, we want the motors to spin in the other direction.
if (PID_R < -10) {
  bool direction_1 = LOW;
  digitalWrite(direction1,direction_1);
}
if (PID_P > 0) {
  bool direction_3 = LOW;
  digitalWrite(direction3,direction_3);
}
if (PID_R > 0 && PID_P > 0 ) {
  bool direction_1 = HIGH;
  digitalWrite(direction1,direction_1);
  bool direction_3 = HIGH;
  digitalWrite(direction3,direction_3);
}

pwmMotor1 =  PID_R;
pwmMotor3 =  PID_P; 

if (pwmMotor3>255) {
  pwmMotor3 = 255;
}
if (pwmMotor1>255) {
  pwmMotor1 = 255;
} 

analogWrite(speed3,pwmMotor3);
analogWrite(speed1,pwmMotor1);

previous_error_Roll = errorRoll; //Remember to store the previous error.
previous_error_Pitch = errorPitch;

}
