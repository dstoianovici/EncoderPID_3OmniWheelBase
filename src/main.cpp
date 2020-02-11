////////////////////////////////////////////////////////////////////////////////
/*

Motor Control code for Nyku 3 Omni Wheel Base

Takes in motor setpoint positions over Serial

Hardware: Cytron 4D04A Motor Driver, 3x Pololu #4867 Motor,
          Arduino Uno, SuperDroid Robotics Triple Encoder Buffer LS7366R

            Useful Info:
            32bit Encoder Reading per channel, 4 counts per quadrature cycle
            4741.44 Count per rotation from Pololu Motor. Approximated to 4741 counts / 2pi


Dan Stoianovici 9/21/19

*/
////////////////////////////////////////////////////////////////////////////////


//Header Files
#include <Arduino.h>
#include <CytronMotorDriver.h>
#include <Serial_Parser.h>
#include <SPI.h>
#include <Encoder_Buffer.h>

//Serial Comms
#define BAUD_RATE 115200
#define DELIM ','
#define NUM_PARAMS 3
#define NUM_MOTORS 3
#define PID_FREQ 60

//Motor Pins
#define mot0_dir 2
#define mot0_en 3

#define mot1_dir 4
#define mot1_en 5

#define mot2_dir 7
#define mot2_en 6




#define range_M 1023 //Max val of pot
#define range_m 0 //Min Val of pot
#define MIDPOINT 512


const uint8_t cs1 = A1; //Chip Select Pins
const uint8_t cs2 = A2;
const uint8_t cs3 = A3;


//Create Serial Parser object
Serial_Parser parser(DELIM,range_M,range_m);

//Create motor objects
CytronMD motor1(PWM_DIR, mot0_en, mot0_dir);
CytronMD motor2(PWM_DIR, mot1_en, mot1_dir);
CytronMD motor3(PWM_DIR, mot2_en, mot2_dir);

//Encoder
Encoder_Buffer enc1(cs1);
Encoder_Buffer enc2(cs2);
Encoder_Buffer enc3(cs3);

//Global Vals for PID control
int setpoints[NUM_PARAMS] = {0,0,0}, setpoints_old[NUM_PARAMS] = {0,0,0};

//PID Vars
float kP[3] = {.5,.5,.5};
float kI[3] = {0,0,0};
// float kI[4] = {0.2,.2,.2};
float kD[3] = {0,0,0};

float deadband = 7.0;


float volatile currentTime[NUM_MOTORS], previousTime[NUM_MOTORS], elapsedTime[NUM_MOTORS];
float volatile error[NUM_MOTORS]={0,0,0}, cumError[NUM_MOTORS]={0,0,0}, rateError[NUM_MOTORS]={0,0,0}, lastError[NUM_MOTORS]={0,0,0};

int param_check[2] = {0,0}; //Checks Values Fetched by SerialParser

const int hist_length = 60;
int iter = 0;

float  cumError_hist0[hist_length];
float  cumError_hist1[hist_length];
float  cumError_hist2[hist_length];
float  cumError_hist3[hist_length];


//PID function Prototype
float computePID(int setpoint, int state, int channel,float _deadband);
float bound(float val, float range);
int Error_Hist(float Error, float* Error_Hist, int hist_size, int _iter, float _deadband);
float average(float* arr, int len);

void setup() {
  Serial.begin(BAUD_RATE);
  SPI.begin();
  enc1.initEncoder();
  enc2.initEncoder();
  enc3.initEncoder();

}

void loop() {
   int PID_flag = 1;
  //int setpoint[NUM_PARAMS]; //array of intergers corresponting to number of parameters neededs
  parser.GetParams(setpoints,param_check); //Pull in setpoints from serial parser

  // Serial.print("Setpoint 1: ");
  // Serial.print(setpoints[0]);
  // Serial.println();
  // Serial.print("Setpoint 2: ");
  // Serial.print(setpoints[1]);
  // Serial.println();
  // Serial.print("Setpoint 3: ");
  // Serial.print(setpoints[2]);
  // Serial.println();
  //
  // Serial.print("Param Check 0: ");
  // Serial.print(param_check[0]);
  // Serial.println();
  // Serial.print("Param Check 1: ");
  // Serial.print(param_check[1]);
  // Serial.println();
  // Serial.println();

  delay(500);
   if(param_check[1] != 0){ // Parameters out of range
     //Serial.println("params out of range");
     for(int i=0;i<NUM_PARAMS;i++){
       setpoints[i] = setpoints_old[i]; // Will reset to midpoint if these
     }
     PID_flag = 0;
   }

   else if(param_check[0] != NUM_PARAMS){
     if(param_check[0] == 0) PID_flag = 1; //If no params recieved
     else{ // Too many params recieved
        //Serial.println("params do not match");
        PID_flag = 0;
      }
   }

   if (PID_flag == 1){//If serial Value are ok
      if(param_check[0] == NUM_PARAMS){ // Correct number of params recieved, anti windup for integral error
        for(int i=0;i<NUM_PARAMS;i++){
             cumError[i] = 0;
        }
      }

      for(int i=0;i<NUM_PARAMS;i++){
           setpoints_old[i] = setpoints[i];
      }


      int out1 = computePID(setpoints[0], (enc1.readEncoder()/4), 0, deadband);
      motor1.setSpeed(-out1);
      int out2 = computePID(setpoints[1], (enc2.readEncoder()/4), 1, deadband);
      motor2.setSpeed(-out2);
      int out3 = computePID(setpoints[2], (enc3.readEncoder()/4), 2, deadband);
      motor3.setSpeed(-out3);

      iter++;
      if(hist_length<=iter) iter = 0;



      // motor1.setSpeed(-out1);
      // motor2.setSpeed(-out2);
      // motor3.setSpeed(-out3);


      Serial.print("SP1: ");
      Serial.println(setpoints[0]);
      Serial.print("State1: ");
      Serial.print(enc1.readEncoder());
      Serial.println();
      Serial.println();


      Serial.print("SP2: ");
      Serial.println(setpoints[1]);
      Serial.print("State2: ");
      Serial.print(enc2.readEncoder());
      Serial.println();
      Serial.println();

      Serial.print("SP3: ");
      Serial.println(setpoints[2]);
      Serial.print("State3: ");
      Serial.print(enc3.readEncoder());
      Serial.println();
      Serial.println();

       delay(1000/PID_FREQ);
     }
  }




///////////////////PID Function////////////////////////////////////////
float computePID(int setpoint, int state, int channel,float _deadband){

  currentTime[channel] = millis();
  elapsedTime[channel] = (currentTime[channel]-previousTime[channel])/1000;

  error[channel] = float(setpoint - state);
  if(abs(error[channel]) <= _deadband){ error[channel] = 0;}
  //else;

  cumError[channel] += error[channel]*elapsedTime[channel];
  rateError[channel] = (error[channel]-lastError[channel])/elapsedTime[channel];

  float out = kP[channel]*error[channel] + kI[channel]*cumError[channel] + kD[channel]*rateError[channel];

  lastError[channel] = error[channel];
  previousTime[channel] = currentTime[channel];

  return -out;
}

float bound(float val, float range){
  float out = val;
  if(abs(val)< range){
    out = 0;
  }
  return out;
}

float average(float* arr, int len){
  float sum = 0;
  for(int i=0; i<len; i++){
    sum += arr[i];
  }
  float avrg = sum/len;
  return avrg;
}

int Error_Hist(float Error, float* Error_Hist, int hist_size, int _iter, float _deadband){
    int flag = 0;
    Error_Hist[_iter] = Error;
    float avg = average(Error_Hist, hist_size);

    if(abs(avg - Error)<deadband) flag = 1;
    else flag = 0;

    return flag;
}
