#include <math.h> // M_PIを使用するために必要
int Motor_PWM[3] = {13,11,9};
int Motor_DIR[3] = {12,10,8};
int speed_pwm = 0; 
int power = 0;
double face_rad = 0.0; 
int i = 0;
int Motor_angle[3] = {60,180,300};
double Motor_rad[3] = {0.0, 0.0, 0.0};

int Motor_rev[3] = {-1,-1,-1};///反転調整 1 or -1

int Line_pin[4] = {A0,A1,A2,A3};
int line_posi[4] = {0,0,0,0};
int line_flag = 0;
int line_threshold[4] = {10,5,5,5}; ///ラインセンサーの閾値
int line_timer = 0;

void setup()
{
  Serial.begin(9600);
  
  for(int i = 0; i < 3; i++){
    pinMode(Motor_PWM[i], OUTPUT);
    pinMode(Motor_DIR[i], OUTPUT);
    
    digitalWrite(Motor_PWM[i], HIGH);
    
    // 角度計算
    Motor_rad[i] = Motor_angle[i] * M_PI / 180.0;
  }
}

///回したいモーターの番号とスピードから出力を決定する関数
void Motor(int Motor_mum ,int speed)
{
    speed = 127 + speed;
    digitalWrite(Motor_PWM[Motor_mum], HIGH); 
    analogWrite(Motor_DIR[Motor_mum], speed);
    Serial.println(speed);


}
void MotorDrive(int face_angle ,int speed_per)
{
  speed_pwm = 128 * speed_per / 100 ; 
  
  face_rad = face_angle * M_PI / 180.0 ; // 180.0 で double 演算を強制

  for (int i = 0; i < 3; i++) {
    // power_float は sin関数の戻り値 (double) を使って計算される
    double power_float = sin(Motor_rad[i] - face_rad) * speed_pwm * Motor_rev[i];
    
    // power を int にキャスト
    power = (int)power_float;

    if (power > 128){
      power = 128;
    } else if(power < -128){
      power = -128;
    }

    Motor(i, power);
    // Serial.println(power);
  }
}

///ラインセンサの値を読み取りラインの位置を判定する関数
void Line_read(){
  line_flag = 0;
  for(int i = 0; i < 4; i++){
    line_posi[i] = 0;
    int line_val = analogRead(Line_pin[i]);
    if (line_val > line_threshold[i]){
      line_flag = 1;
      line_posi[i] = 1;
    } 
}}

/////////////////////////////////////////////Main Loop//////////////////////////////////////////////////

//  MotorDrive(進行方向(0~360°),速度(0~100 %)

void loop()
{
  while(1){
  Line_read();
  if (line_flag == 1){          //ラインが検知されてるとき

    if (line_posi[0] == 1){         //前が反応したとき
      MotorDrive(180,60);
      line_timer = 5;
    }
    else if(line_posi[1] == 1){    //右が反応したとき
      MotorDrive(270,60);
      line_timer = 5;
    }
    else if(line_posi[2] == 1){    //後ろが反応したとき
      MotorDrive(0,60);
      line_timer = 5;
    }
    else if(line_posi[3] == 1){    //左が反応したとき
      MotorDrive(90,60);
      line_timer = 5;
    }

  }else if (line_timer > 0){
    line_timer = line_timer -1 ;
  }else if (line_timer == 0){
  MotorDrive(0, 70); //ラインが反応していない時、かつラインタイマーが0の時にようやく実行される。
  }
  Serial.println("------------------------------------");
}}