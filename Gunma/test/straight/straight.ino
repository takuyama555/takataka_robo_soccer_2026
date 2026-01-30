#include <math.h> 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define gryo_p 0.8 

//////// GAME MODE (1:デバッグ表示あり, 0:本番用) //////////
int game_mode = 0;

// 回り込みのための計算式の係数
#define CIRC_BASE pow(0.6, 1.0 / 22.0)
#define CIRC_WEIGHT 3.5

#define STRAIGHT_SPEED 60
#define CIRC_SPEED     50

int move_angle = 0;

/////// ボタン関連
const int buttonOn_Pin = 52;   
const int buttonOff_Pin = 53;  
bool game_flag = 0;            // 1: ゲーム中 (ボール追従)
int game_start = 0;            // 1: スタート準備中 (姿勢制御のみ)

// タイムアウト設定 (ms)
const unsigned long TIMEOUT_MS = 5;

// --- 変数定義 ---
double print[32];
int Motor_PWM[4] = {4,6,3,7};
int Motor_DIR[4] = {2,8,5,9};
int speed_pwm = 0; 
double face_rad = 0.0; 
int Motor_angle[4] = {45, 135, 225, 315};
double Motor_rad[4] = {0.0, 0.0, 0.0, 0.0};
int Motor_rev[4] = {1.2, 1, 1, -1}; // 反転調整
double power[4] = {0.0, 0.0, 0.0, 0.0};
int speed = 0; // グローバル変数を0で初期化

// --- ジャイロ変数 ---
MPU6050 mpu;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
double current_yaw = 0.0; 

int ir_flag = 0;   // 0:ボールなし, 1:ボールあり
int ir_angle = 0;  // 0〜360度
int ir_dist = 255; // 0:近い 〜 255:遠い

int line_flag = 0;   // 0:ラインなし, 1:ラインあり
int line_angle = 0;  // ラインの角度
byte sensor_raw[4];  
int line_check[32];

// ==========================================
// ジャイロ初期化
// ==========================================
void setupMPU() {
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-1811);
  mpu.setYAccelOffset(-1108);
  mpu.setZAccelOffset(960);
  mpu.setXGyroOffset(72);
  mpu.setYGyroOffset(-9);
  mpu.setZGyroOffset(0);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("DMP Initialization failed.");
  }
}

// ==========================================
// ジャイロ値取得
// ==========================================
double getYawPitchRoll() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    double mpu_degree = ypr[0] * 180 / M_PI;
    print[18] = mpu_degree;
    current_yaw = mpu_degree; 
  }
  return current_yaw; 
}

// ==========================================
// Setup
// ==========================================
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  setupMPU();
  
  pinMode(buttonOn_Pin, INPUT_PULLUP);
  pinMode(buttonOff_Pin, INPUT_PULLUP);

  // タイマー設定 (Arduino Mega前提)
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;  
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;  
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;  
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;  

  for(int i = 0; i < 4; i++){
    pinMode(Motor_PWM[i], OUTPUT);
    pinMode(Motor_DIR[i], OUTPUT);
    digitalWrite(Motor_PWM[i], HIGH);
    analogWrite(Motor_DIR[i], 127); // 初期停止
    Motor_rad[i] = Motor_angle[i] * M_PI / 180.0;
  }
    for (int i = 0; i < 4; i++) {

    digitalWrite(Motor_PWM[i], HIGH);

    analogWrite(Motor_DIR[i], 127);

  }
}

// ==========================================
// モーター出力
// ==========================================
void Motor(int Motor_num, int motor_speed) // 引数名変更
{
    // motor_speedは 0〜127 程度の大きさ
    int pwm_val = 127 + motor_speed;
    if (pwm_val > 255) pwm_val = 255;
    if (pwm_val < 0)   pwm_val = 0;

    digitalWrite(Motor_PWM[Motor_num], HIGH); 
    analogWrite(Motor_DIR[Motor_num], pwm_val); 
}

// ==========================================
// 移動関数 
// ==========================================
void MotorDrive(int face_angle, int speed_per, int gryo_val)
{
  speed_pwm = 127 * speed_per / 100; 
  face_rad = face_angle * M_PI / 180.0; 

  for (int i = 0; i < 4; i++) {
    power[i] = (sin(Motor_rad[i] - face_rad) * speed_pwm + gryo_val) * Motor_rev[i];  
  }

  double max_val = 0.0;
  for (int i = 0; i < 4; i++){
    if (abs(power[i]) > max_val){
      max_val = abs(power[i]);
    }
  }

  double ratio = 1.0;
  if (max_val > 127.0){
    ratio = 127.0 / max_val;
  }

  for(int i = 0; i < 4; i++){
    int final_power = (int)(power[i] * ratio);
    Motor(i, final_power);
  }
}


// ==========================================
// Main Loop
// ==========================================
void loop()
{
  // ジャイロ更新
  double gryo_val = getYawPitchRoll() * gryo_p ;
  MotorDrive(0, 60, gryo_val); 
}
