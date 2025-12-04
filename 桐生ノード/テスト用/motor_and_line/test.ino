#include <math.h> 
int Motor_PWM[3] = {13,11,9};
int Motor_DIR[3] = {12,10,8};
int speed_pwm = 0; 
int power = 0;
double face_rad = 0.0; 
int i = 0;
int Motor_angle[3] = {60,180,300};
double Motor_rad[3] = {0.0, 0.0, 0.0};
int Motor_rev[3] = {1,1,1};///反転調整 1 or -1

int Line_pin[4] = {A0,A1,A2,A3};
int line_posi[4] = {0,0,0,0};
int line_flag = 0;
int line_threshold = 5;

void setup()
{
  Serial.begin(9600);
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  for(int i = 0; i < 3; i++){
    Motor_rad[i] = Motor_angle[i] * M_PI / 180.0; // 180.0 で double 演算を強制
    Serial.println(Motor_rad[i]);
  }
}

///回したいモーターの番号とスピードから出力を決定する関数
void Motor(int Motor_mum ,int speed)
{
  if (speed < 0){
    speed = -1*speed;
    digitalWrite(Motor_DIR[Motor_mum], HIGH); // 逆転 DIR=1
  } else if(speed >= 0){
    digitalWrite(Motor_DIR[Motor_mum], LOW); // 正転 DIR=0
  }
  // speed は 0〜255 の範囲であるべき
  analogWrite(Motor_PWM[Motor_mum], speed);
}


void MotorDrive(int face_angle ,int speed_per)
{
  speed_pwm = 255 * speed_per / 100 ; 
  
  face_rad = face_angle * M_PI / 180.0 ; // 180.0 で double 演算を強制

  for (int i = 0; i < 3; i++) {
    // power_float は sin関数の戻り値 (double) を使って計算される
    double power_float = sin(Motor_rad[i] - face_rad) * speed_pwm * Motor_rev[i];
    power = (int)power_float;

    if (power > 255){
      power = 255;
    } else if(power < -255){
      power = -255;
    }

    Motor(i, power);
    Serial.println(power);
  }
}

///ラインセンサの値を読み取りラインの位置を判定する関数
void Line_read(){
  line_flag = 0;
  for(int i = 0; i < 1; i++){
    line_posi[i] = 0;
    int line_val = analogRead(Line_pin[i]);
    if (line_val > line_threshold){
      line_flag = 1;
      line_posi[i] = 1;
    } 
}}




/////////////////////////////////////////////Main Loop//////////////////////////////////////////////////

//  MotorDrive(進行方向(0~360°),速度(0~100 %)

void loop()
{
  Line_read();
  if (line_flag == 1){          //ラインが検知されてるとき

    if (line_posi[0] == 1){         //前が反応したとき
      MotorDrive(180,80);
    }
    else if(line_posi[1] == 1){    //右が反応したとき
      MotorDrive(270,80);
    }
    else if(line_posi[2] == 1){    //後ろが反応したとき
      MotorDrive(0,80);
    }
    else if(line_posi[3] == 1){    //左が反応したとき
      MotorDrive(90,80);
    }

  }else{
  MotorDrive(0, 30);
  }
  Serial.println("------------------------------------");
}