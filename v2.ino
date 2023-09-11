#include <Wire.h>
#include <MPU6050_light.h>
#include<math.h>
#include <util/atomic.h>
#include <stdio.h>

//pins
//left
#define ENCA_l 2
#define ENCB_l 4
#define PWM_l 11
#define IN1 12
#define IN2 13
//right
#define ENCA_r 3
#define ENCB_r 5
#define PWM_r 10
#define IN3 9
#define IN4 8

//sensor
MPU6050 imu(Wire);
void setup() {
    Serial.begin(9600);
    // put your setup code here, to run once:
    // pin settings
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENCA_l, INPUT);
    pinMode(ENCB_l, INPUT);
    pinMode(PWM_l, OUTPUT);

    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENCA_r, INPUT);
    pinMode(ENCB_r, INPUT);
    pinMode(PWM_r, OUTPUT);
    
    Wire.begin();
    imu.begin();
    imu.calcOffsets();

    attachInterrupt(digitalPinToInterrupt(ENCA_l),readEncoderL,RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA_r),readEncoderR,RISING);
}

float pwmbalance;
enum direct{
  left,
  right,
  toward,
  afterward,
};

void loop() {
    // put your main code here, to run repeatedly:
    imu.update();
    float Angle = imu.getAngleX();
    float gyro =  imu.getGyroX();
    int turn = 0;
    float target = 0;
    
//    if(Serial.available()){
//      int order = Serial.read();
//      switch(order){
//        case left:
//          turn = 5;
//          break;
//        case right:
//          turn = -5;
//          break; 
//        case toward:
//          target = 5;
//          break;
//        case afterward:
//          target = -5;
//          break;           
//      }
//    }
    //target 平衡繳,Angle 旋轉軸,gyro 角速度
    float anglespeed = anglebalance(target,Angle,gyro);
    float motorspeed = motorbalance(calculateENC());
    pwmbalance = anglespeed - motorspeed;
    SpeedControl(pwmbalance,turn);
}

//馬達pwr<255,kpangle*theta(-20~20)<255
//kpangle<127.5
float kpangle = 8;
float kdangle = 0.075;

float anglespeed ;
float anglebalance(float targrt,float Angle,float gyro){
  anglespeed = kpangle*(Angle-targrt) + kdangle*gyro;

  return anglespeed;
}

float kpmotor = 0.21;
float kimotor = 0.0002;
int encoder_least = 0;
float encoder = 0;
float encoder_integral = 0;
float motorspeed;
float motorbalance(int encoder_least){
  encoder *= 0.8;
  encoder += encoder_least*0.3;
  encoder_integral += encoder;
  if(encoder_integral>1000) encoder_integral=1000;
  if(encoder_integral<-1000) encoder_integral=-1000;

  motorspeed = encoder *kpmotor + encoder_integral*kimotor;
  return motorspeed;
}

//////////////////////////////////////////////////////////////
//speed
// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float eintegral = 0;

int SpeedControl(float vt,int turn) {
  // read the position and velocity
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT; 

  // Convert count/s to RPM
  float v1 = velocity1/517.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;

  // Compute the control signal u
  float kp = 7;
  float ki = 1.5;
  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;

  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  setMotor(dir,pwr+turn,PWM_l,IN1,IN2);
  setMotor(dir,pwr-turn,PWM_r,IN3,IN4);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

volatile int pos_iL = 0;
volatile int pos_iR = 0;
int posPreL = 0;
int posPreR = 0;
long long int ticks = 0;

void readEncoderL(){
  // Read encoder B when ENCA rises
  ticks = 0;
  if(digitalRead(ENCB_l)){
    // If B is high, ticks forward
    ticks --;
  }
  else{
    // Otherwise, ticks backward
    ticks ++;
  }
  pos_iL = pos_iL + ticks;
}

void readEncoderR(){
  // Read encoder B when ENCA rises
  ticks = 0;
  if(digitalRead(ENCB_r)){
    // If B is high, ticks forward
    ticks --;
  }
  else{
    // Otherwise, ticks backward
    ticks ++;
  }
  pos_iR = pos_iR + ticks;
}
int calculateENC(){
  int posL = 0,posR = 0;
  noInterrupts();
  posL = pos_iL;
  posR = pos_iR;
  interrupts();
  int enctotal_L = posL - posPreL;
  int enctotal_R = posR - posPreR;
  posPreL = posL;
  posPreR = posR;
  delay(3);
  
  return enctotal_L-enctotal_R;
}