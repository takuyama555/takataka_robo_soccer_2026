#include <math.h> // M_PIを使用するために必要
#define G2_PWM 13
#define G2_DIR 11
int Motor_PWM[3] = {13,11,9};
int Motor_DIR[3] = {12,10,8};
int speed_pwm = 0; 
int power = 0;
double face_rad = 0.0; 
int i = 0;
int Motor_angle[3] = {60,180,300};
double Motor_rad[3] = {0.0, 0.0, 0.0};
int Motor_rev[3] = {1,1,1};///反転調整 1 or -1

int Line_pin[4] = {0,1,2,3};

void setup()
{
  Serial.begin(9600);
  // pin9,10 PWM周波数を変える -> 31372.55 Hz
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  pinMode(G2_DIR, OUTPUT);
  analogWrite(G2_PWM, 0);

  // Motor_rad の計算を double で実行
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
  } else {
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
    
    // power を int にキャスト
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


/////////////////////////////////////////////Main Loop//////////////////////////////////////////////////

//  MotorDrive(進行方向(0~360°),速度(0~100 %)

void loop()
{
  MotorDrive(0, 30);
  Serial.println("------------------------------------");
}