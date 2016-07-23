#include "MsTimer2.h"
#include "I2Cdev.h"
#include <SoftwareSerial.h>

const unsigned char safedistance = 0;  //安全距离，mm
const unsigned char straight = 100;   //人工操控向前时长，ms
const unsigned char turntime = 100;   //人工操控转向时长，ms
const int x_safe = 0;   //安全距离
const int lateral_safe = 0;   //侧向障碍通过阈值距离
float kp_s = 0.045, ki_s = 0, kd_s = 0.5;   //轮速PID参数
float kp_t, ki_t, kd_t;   //转角PID参数

unsigned char theta_t = 0;        //理论航向角，度数
int leftspeed_t = 0;    //左轮理论轮速，rpm
int rightspeed_t = 0;   //右轮理论轮速，rpm
unsigned char theta_r = 0;        //实际航向角，度数
int leftspeed_r = 0;    //左轮实际轮速，rpm
int rightspeed_r = 0;   //右轮实际轮速，rpm
int obstacle[] = {0, 0, 0, 0};   //避障检测情况，mm
boolean control = true;   //是否人工操控
unsigned char command = 0;    //人工操控信号

SoftwareSerial camera(50, 51);
int i = 0;    //定时器中断内部参数
int j = 0;
int fRErr_p_ls = 0;   //储存误差
int fRErr_p_rs = 0;
int fRErr_p_a = 0;
boolean bTag_30ms = false;   //中断标志
boolean bTag_50ms = false;
unsigned long obstacle1_1;    //避障传感器检测中间变量
unsigned long obstacle1_2;
unsigned long obstacle2_1;
unsigned long obstacle2_2;
unsigned long obstacle3_1;
unsigned long obstacle3_2;
unsigned long obstacle4_1;
unsigned long obstacle4_2;
unsigned char ad_value1=0;  //红外，0为offline，1为online
unsigned char ad_value2=0;
unsigned char ad_value3=0;
unsigned char ad_value4=0;
int speedsensor_l[]={0, 0, 0, 0, 0};
int speedsensor_r[]={0, 0, 0, 0, 0};
int sum_l;
int sum_r;
int out_l = 0;
int out_r = 0;

void timer(){
  i+=1;
  if(i%30==0){
    bTag_30ms = true;
  }
  if(i%50==0){
    bTag_50ms = true;
  }
  if(i==10000){
    i = 0;
  }
}

void setup() {  
  Serial.begin(9600);
  camera.begin(9600);
  pinMode(8, OUTPUT);   //左轮PWM  左轮A,右轮B
  pinMode(7, OUTPUT);   //右轮PWM
  pinMode(24, OUTPUT);    //左轮IN1
  pinMode(26, OUTPUT);    //左轮IN2
  pinMode(28, OUTPUT);    //右轮IN1
  pinMode(30, OUTPUT);    //右轮IN2
  pinMode(22, INPUT_PULLUP);   //左轮编码器
  pinMode(23, INPUT_PULLUP);   //右轮编码器
  pinMode(18, INPUT);    //避障传感器1接收
  pinMode(19, INPUT);    //避障传感器2接收
  pinMode(2, INPUT);    //避障传感器3接收
  pinMode(3, INPUT);    //避障传感器4接收
  pinMode(31, OUTPUT);    //避障传感器1发射
  pinMode(32, OUTPUT);    //避障传感器2发射
  pinMode(33, OUTPUT);    //避障传感器3发射
  pinMode(34, OUTPUT);    //避障传感器4发射
  pinMode(36,INPUT);      //红外1接收
  pinMode(37,INPUT);      //红外2接收
  pinMode(38,INPUT);      //红外3接收
  pinMode(39,INPUT);      //红外4接收
  digitalWrite(24, LOW);
  digitalWrite(26, LOW);
  digitalWrite(28, LOW);
  digitalWrite(30, LOW);
  MsTimer2::set(1, timer);
  MsTimer2::start();
}

void pick(){    //抓取
}

void speedPID(){
  int fRErr_c_ls = 0;
  int fDErr_c_ls = 0;
  int fIntVol_ls = 0;
  unsigned int fCtrVol_ls = 0;
  fRErr_c_ls = leftspeed_t - leftspeed_r;
  fDErr_c_ls = fRErr_c_ls - fRErr_p_ls;
  fRErr_p_ls = fRErr_c_ls;
  fCtrVol_ls = kp_s * fRErr_c_ls;
  fCtrVol_ls += ki_s * fIntVol_ls;
  fCtrVol_ls += kd_s * fDErr_c_ls;
  out_l += fCtrVol_ls;
  if(out_l > 255){
    out_l = 255;
  }
  if(out_l < 0){
    out_l = 0;
  }
  analogWrite(8, out_l);
  int fRErr_c_rs = 0;
  int fDErr_c_rs = 0;
  int fIntVol_rs = 0;
  unsigned int fCtrVol_rs = 0;
  fRErr_c_rs = rightspeed_t - rightspeed_r;
  fDErr_c_rs = fRErr_c_rs - fRErr_p_rs;
  fRErr_p_rs = fRErr_c_rs;
  fCtrVol_rs = kp_s * fRErr_c_rs;
  fCtrVol_rs += ki_s * fIntVol_rs;
  fCtrVol_rs += kd_s * fDErr_c_rs;
  out_r += fCtrVol_rs;
  if(out_r > 255){
    out_r = 255;
  }
  if(out_r < 0){
    out_r = 0;
  }
  analogWrite(7, out_r);
}

void humancontrol(){    //人工操作
  if(camera.available()){
    char inByte = camera.read();
    if(inByte == 'w'){
      command = 1;
    }
    if(inByte == 's'){
      command = 2;
    }
    if(inByte == 'a'){
      command = 3;
    }
    if(inByte == 'd'){
      command = 4;
    }
    if(inByte == 'p'){
      command = 5;
    }
    if(inByte == 'q'){
      command = 6;
    }
  }
  switch(command){
    case 1: 
      Serial.println("1");
      digitalWrite(24, HIGH);
      digitalWrite(26, LOW);
      digitalWrite(28, HIGH);
      digitalWrite(30, LOW);
      analogWrite(8, 150);
      analogWrite(7, 150);
      delay(straight);
      analogWrite(8, 0);
      analogWrite(7, 0);
      digitalWrite(24, LOW);
      digitalWrite(26, LOW);
      digitalWrite(28, LOW);
      digitalWrite(30, LOW);
      break;
    case 2:
      Serial.println("2");
      digitalWrite(24, LOW);
      digitalWrite(26, HIGH);
      digitalWrite(28, LOW);
      digitalWrite(30, HIGH);
      analogWrite(8, 150);
      analogWrite(7, 150);
      delay(straight);
      analogWrite(8, 0);
      analogWrite(7, 0);
      digitalWrite(24, LOW);
      digitalWrite(26, LOW);
      digitalWrite(28, LOW);
      digitalWrite(30, LOW);
      break;
    case 3: 
      Serial.println("3");
      digitalWrite(24, HIGH);
      digitalWrite(26, LOW);
      digitalWrite(28, LOW);
      digitalWrite(30, HIGH);
      analogWrite(8, 150);
      analogWrite(7, 150);
      delay(turntime);
      analogWrite(8, 0);
      analogWrite(7, 0);
      digitalWrite(24, LOW);
      digitalWrite(26, LOW);
      digitalWrite(28, LOW);
      digitalWrite(30, LOW);
      break;
    case 4:
      Serial.println("4");
      digitalWrite(24, LOW);
      digitalWrite(26, HIGH);
      digitalWrite(28, HIGH);
      digitalWrite(30, LOW);
      analogWrite(8, 150);
      analogWrite(7, 150);
      delay(turntime);
      analogWrite(8, 0);
      analogWrite(7, 0);
      digitalWrite(24, LOW);
      digitalWrite(26, LOW);
      digitalWrite(28, LOW);
      digitalWrite(30, LOW);
      break;
    case 5: 
      Serial.println("5");
      pick();
      break;
    case 6:
      control = false;
      digitalWrite(24, HIGH);
      digitalWrite(26, LOW);
      digitalWrite(28, HIGH);
      digitalWrite(30, LOW);
      break;
    default:
      break;
  }
  command = 0;
}

void function1_1(){   //传感器上升沿中断
  obstacle1_1 = micros();
  detachInterrupt(digitalPinToInterrupt(18));
  attachInterrupt(digitalPinToInterrupt(18), function1_2, FALLING);
}

void function1_2(){   //传感器下降沿中断
  obstacle1_2 = micros();
  obstacle[0] = (obstacle1_2-obstacle1_1)/5.88;
  detachInterrupt(digitalPinToInterrupt(18));
}

void function2_1(){
  obstacle2_1 = micros();
  detachInterrupt(digitalPinToInterrupt(19));
  attachInterrupt(digitalPinToInterrupt(19), function2_2, FALLING);
}

void function2_2(){
  obstacle2_2 = micros();
  obstacle[1] = (obstacle2_2-obstacle2_1)/5.88;
  detachInterrupt(digitalPinToInterrupt(19));
}

void function3_1(){
  obstacle3_1 = micros();
  detachInterrupt(digitalPinToInterrupt(2));
  attachInterrupt(digitalPinToInterrupt(2), function3_2, FALLING);
}

void function3_2(){
  obstacle3_2 = micros();
  obstacle[2] = (obstacle3_2-obstacle3_1)/5.88;
  detachInterrupt(digitalPinToInterrupt(2));
}

void function4_1(){
  obstacle4_1 = micros();
  detachInterrupt(digitalPinToInterrupt(3));
  attachInterrupt(digitalPinToInterrupt(3), function4_2, FALLING);
}

void function4_2(){
  obstacle4_2 = micros();
  obstacle[3] = (obstacle4_2-obstacle4_1)/5.88;
  detachInterrupt(digitalPinToInterrupt(3));
}

void sensor(){
  unsigned long time1 = pulseIn(22, HIGH, 3000);
  unsigned long time2 = pulseIn(23, HIGH, 3000);
  speedsensor_l[4] = speedsensor_l[3];
  speedsensor_l[3] = speedsensor_l[2];
  speedsensor_l[2] = speedsensor_l[1];
  speedsensor_l[1] = speedsensor_l[0];
  speedsensor_r[4] = speedsensor_r[3];
  speedsensor_r[3] = speedsensor_r[2];
  speedsensor_r[2] = speedsensor_r[1];
  speedsensor_r[1] = speedsensor_r[0];
  speedsensor_l[0] = 160333.5/time1;
  speedsensor_r[0] = 160333.5/time2;
  sum_l=0;
  sum_r=0;
  int k;
  for(k=0; k<5; k++){
    sum_l+=speedsensor_l[k];
    sum_r+=speedsensor_r[k];
  }
  leftspeed_r = sum_l/5;
  rightspeed_r = sum_r/5;
  digitalWrite(31, LOW);
  digitalWrite(32, LOW);
  digitalWrite(33, LOW);
  digitalWrite(34, LOW);
  delayMicroseconds(2);
  digitalWrite(31, HIGH);
  digitalWrite(32, HIGH);
  digitalWrite(33, HIGH);
  digitalWrite(34, HIGH);
  delayMicroseconds(20);
  digitalWrite(31, LOW);
  digitalWrite(32, LOW);
  digitalWrite(33, LOW);
  digitalWrite(34, LOW);
  attachInterrupt(digitalPinToInterrupt(18), function1_1, RISING);
  attachInterrupt(digitalPinToInterrupt(19), function2_1, RISING);
  attachInterrupt(digitalPinToInterrupt(2), function3_1, RISING);
  attachInterrupt(digitalPinToInterrupt(3), function4_1, RISING);
}

void strategy(){
  if(obstacle[0] < x_safe){
    digitalWrite(24, LOW);
    digitalWrite(26, LOW);
    digitalWrite(28, LOW);
    digitalWrite(30, LOW);
    delay(100);
    digitalWrite(24, HIGH);
    digitalWrite(26, LOW);
    digitalWrite(28, LOW);
    digitalWrite(30, HIGH);
    while(obstacle[0] < x_safe){
      leftspeed_t = 100;
      rightspeed_t = 100;
    }
    digitalWrite(24, LOW);
    digitalWrite(26, LOW);
    digitalWrite(28, LOW);
    digitalWrite(30, LOW);
    delay(100);
    digitalWrite(24, LOW);
    digitalWrite(26, HIGH);
    digitalWrite(28, LOW);
    digitalWrite(30, HIGH);
    leftspeed_t = 100;
    rightspeed_t = 100;
  }
  if(obstacle[1] < x_safe){
    digitalWrite(24, LOW);
    digitalWrite(26, LOW);
    digitalWrite(28, LOW);
    digitalWrite(30, LOW);
    delay(100);
    digitalWrite(28, HIGH);
    digitalWrite(30, LOW);
    digitalWrite(24, LOW);
    digitalWrite(26, HIGH);
    while(obstacle[1] < x_safe){
      leftspeed_t = 100;
      rightspeed_t = 100;
    }
    digitalWrite(24, LOW);
    digitalWrite(26, LOW);
    digitalWrite(28, LOW);
    digitalWrite(30, LOW);
    delay(100);
    digitalWrite(24, LOW);
    digitalWrite(26, HIGH);
    digitalWrite(28, LOW);
    digitalWrite(30, HIGH);
    leftspeed_t = 100;
    rightspeed_t = 100;
  }
/*    int flag = 0;
    while(flag ==0 || obstacle[3]< lateral_safe){
      if(obstacle[3] < lateral_safe){
        flag = 1;
      }
      else{
        leftspeed_t = 100;
        rightspeed_t = 100;
      }
    }
    digitalWrite(24, LOW);
    digitalWrite(26, LOW);
    digitalWrite(28, LOW);
    digitalWrite(30, LOW);
    delay(100); */
}

void IMU(){
}

void controlsense(){
  if(camera.available()){
    if(camera.read() == 'c'){
      control = true;
      digitalWrite(24, LOW);
      digitalWrite(26, LOW);
      digitalWrite(28, LOW);
      digitalWrite(30, LOW);
    }
  }
}

void loop(){
  controlsense();
  while(control == true){
    humancontrol();
  }
  strategy();
  if(bTag_50ms==true){
    sensor();
    speedPID();
    bTag_50ms = false;
  }
  if(bTag_30ms==true){
    IMU();
    bTag_30ms = false;
  }
}
