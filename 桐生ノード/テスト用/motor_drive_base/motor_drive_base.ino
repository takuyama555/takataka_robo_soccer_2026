/*
 Name:    DDK6ch_MD_HAIRETSU.ino
 Created: 2021/03/15 1:01:04
 Author:  Administrator

ダイセン電子工業製4CHモーターコントローラーDSR1202用
パワーコントロール
前進 1 ～ 100
後進 -1 ～ -100
停止 0
電磁ブレーキ 1000
※I2Cの通信は"6CHモーターコントローラーDSR631"と共通化されている為、5CH,6CHは空送りしています
*/

#include <Wire.h>
volatile byte address = 0x14;
char Motor_Power[6] = { 0,0,0,0,0,0 };
int Motor_angle[3] = {60,180,300};
int Motor_rad[3] = {0,0,0};
int Motor_rev[3] = {1,1,1};
int power = 0;
int face_rad = 0;
#define PI 3.141592653589793
int i = 0;

void setup()
{
  Wire.begin();             
}



void Motor(int num, int val)          // モータ制御用変数設定 ( 0～5ch,パワー )
{
  if (val == 1000)
  {
    Motor_Power[num] = 0x80;
  }
  else
  {
    constrain(val, -100, 100);
    if (val < 0)
    {
      val = abs(val);
      bitWrite(val, 7, 1);
      Motor_Power[num] = val;
    }
    else
    {
      Motor_Power[num] = val;
    }
  }
}

void MotorDrive(int face_angle ,int speed)
{
  for (int i = 0; i < 3; i++) {
    Motor_rad[i] = Motor_angle[i] * PI / 180;
    face_rad = face_angle * PI / 180;
    power = sin(face_rad - Motor_rad[i])*speed * Motor_rev[i];
    if (power > 100){
      power = 100;
    }
    Motor(i, power);
  }
  Wire.beginTransmission(address >> 1);
  Wire.write(Motor_Power, 6);
  Wire.endTransmission();
  delay(1);
}
/////////////////////////////////////////////Main Loop//////////////////////////////////////////////////

//  MotorDrive(進行方向(0~360°),速度(0~100 %)

void loop()
{
  MotorDrive(0, 100);

  
}