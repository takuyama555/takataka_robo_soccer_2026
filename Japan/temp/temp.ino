#include <math.h> 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define gryo_p 0.8 

//////// GAME MODE (1:デバッグ表示あり, 0:本番用) //////////
int game_mode = 0;

// 回り込みのための計算式の係数
#define CIRC_BASE pow(0.6, 1.0 / 20.0)
#define CIRC_WEIGHT 3.5

#define STRAIGHT_SPEED 60
#define CIRC_SPEED     50

/////// ボタン関連
const int buttonOn_Pin = 52;   
const int buttonOff_Pin = 53;  
bool game_flag = 0;            // 1: ゲーム中 (ボール追従)
int game_start = 0;            // 1: スタート準備中 (姿勢制御のみ)

// タイムアウト設定 (ms)
const unsigned long TIMEOUT_MS = 5;

// --- 変数定義 ---
double print[32];
int Motor_PWM[4] = {8,4,9,5};
int Motor_DIR[4] = {6,2,7,3};
int speed_pwm = 0; 
double face_rad = 0.0; 
int Motor_angle[4] = {45, 135, 225, 315};
double Motor_rad[4] = {0.0, 0.0, 0.0, 0.0};
double Motor_rev[4] = {1, 1, -1, 1}; // 反転調整
double power[4] = {0.0, 0.0, 0.0, 0.0};
int speed = 0; // グローバル変数を0で初期化
int recent_line_angle[10] = {999,999,999,999,999,999,999,999,999,999};
int line_time = 0;
bool yellow_flag = 0;
int  yellow_angle = 0;
bool blue_flag   = 0;
int  blue_angle   = 0;
int goal_angle = 0;
int yellow_height = 0;
int blue_height   = 0;
int goal_height   = 0;

bool btn_off_holding   = false;
unsigned long btn_off_hold_start = 0;
bool yellow_court = true;  // true = Yellow, false = Blue

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

  // ★あなたのロボット固有の値
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
// IR取得 
// ==========================================
void ir_read(){
  while (Serial2.available()) Serial2.read();
  Serial2.write(255); 

  unsigned long start_time = millis();
  while (Serial2.available() < 5) {
    if (millis() - start_time > TIMEOUT_MS) return; 
  }

  if (Serial2.read() == 255) {
    byte raw_flag = Serial2.read();
    byte raw_ang_low = Serial2.read();
    byte raw_ang_high = Serial2.read();
    byte raw_dist = Serial2.read();

    ir_flag = raw_flag;
    ir_angle = raw_ang_low | (raw_ang_high << 7);
    ir_dist = raw_dist;
  }
} 

// ==========================================
// ライン取得
// ==========================================
void line_read(){
  while (Serial1.available()) Serial1.read();
  
  Serial1.write(254); 

  unsigned long start_time = millis();
  while (Serial1.available() < 8) {
    if (millis() - start_time > TIMEOUT_MS) return;
  }

  if (Serial1.read() == 254) { 
    byte r_flag = Serial1.read();       
    byte r_ang_low = Serial1.read();    
    byte r_ang_high = Serial1.read();   
    
    byte s0_7   = Serial1.read();       
    byte s8_15  = Serial1.read();       
    byte s16_23 = Serial1.read();       
    byte s24_31 = Serial1.read();       

    line_flag = r_flag;
    line_angle = r_ang_low | (r_ang_high << 7);
    
    sensor_raw[0] = s0_7;
    sensor_raw[1] = s8_15;
    sensor_raw[2] = s16_23;
    sensor_raw[3] = s24_31;
  }

  for (int i = 0; i < 4; i++) {       
    for (int bit = 0; bit < 8; bit++) { 
      int sensor_num = i * 8 + bit;
      if ( (sensor_raw[i] >> bit) & 1 ) {
        line_check[sensor_num] = 1;
      } else {
        line_check[sensor_num] = 0;
      }
    }
  }
}

// ==========================================
// ゴール取得 
// ==========================================
void camera_read() {
    // 1. リクエスト送信 (253)
    Serial3.write(253); 

    // 2. データの到着を待つ (タイムアウト付き) 
    uint32_t startTime = millis();
    while (Serial3.available() < 11) {
        if (millis() - startTime > 5) return;
    }

    // 3. パケットの読み取り
    uint8_t header = Serial3.read();
    
    if (header == 253) { 
        yellow_flag = (Serial3.read() == 1);
        uint8_t y_low  = Serial3.read();
        uint8_t y_high = Serial3.read();
        yellow_angle = y_low | (y_high << 7);
        uint8_t yh_low  = Serial3.read(); // ★ 黄色の高さ
        uint8_t yh_high = Serial3.read();
        yellow_height = yh_low | (yh_high << 7);

        blue_flag = (Serial3.read() == 1);
        uint8_t b_low  = Serial3.read();
        uint8_t b_high = Serial3.read();
        blue_angle = b_low | (b_high << 7);
        uint8_t bh_low  = Serial3.read(); // ★ 青色の高さ
        uint8_t bh_high = Serial3.read();
        blue_height = bh_low | (bh_high << 7);
        
        // --- 判定ロジック：正面に近い方を goal_angle に採用 ---
        goal_angle  = 0;
        goal_height = 0;

        if (yellow_court == true) { // 黄色ゴール攻めの時
            if(yellow_flag == 1){
                goal_angle  = yellow_angle;
                goal_height = yellow_height;
            }else if(blue_flag == 1){
                goal_angle  = blue_angle-180;
                goal_height = 1;
            }else{
                goal_angle  = 0;
                goal_height = 0; 
            }
        }else{  // 青色ゴール攻めの時
            if(blue_flag == 1){
                goal_angle  = blue_angle;
                goal_height = blue_height;
            }else if(yellow_flag == 1){
                goal_angle  = yellow_angle-180;
                goal_height = 1;
            }else{
                goal_angle  = 0;
                goal_height = 0;  
            }
        }
    }
}


// ==========================================
// Main Loop
// ==========================================
void loop()
{
  // ジャイロ更新
  double gryo_val = getYawPitchRoll() * gryo_p ;
  
  speed = 0; 

  if (game_flag == 1){
    line_read();
    ir_read(); 
    camera_read();
    if (line_flag == 1){
      recent_line_angle[line_time] = line_angle;
      if(line_time > 0){
      if(recent_line_angle[line_time] > 180 && recent_line_angle[line_time - 1] < 180){
        line_angle = recent_line_angle[line_time - 1];
      }
      if(recent_line_angle[line_time] < 180 && recent_line_angle[line_time - 1] > 180){
        line_angle = recent_line_angle[line_time - 1];
      }
    }
      MotorDrive(line_angle + 180, 200, gryo_val);
    if(line_time < 10){
      line_time ++;
    }else{
      line_time = 0;
    }
      delay(30);
    }
    else if(ir_flag == 1){

        int move_angle = 0;
        int min_pow = 15;
        
        // --- 写真の数式に基づくスピード計算の追加 ---
        if (ir_angle >= 0 && ir_angle <= 90) {
            // 0度 ~ 90度の場合 (左): V = (theta / 90)^2 * 100
            speed = min_pow + pow(ir_angle / 90.0, 1.5) * (150 - min_pow);
        } else if (ir_angle >= 270 && ir_angle <= 360) {
            // 270度 ~ 360度の場合 (右): V = ((theta - 360) / 90)^2 * 100
            speed = min_pow + pow(abs(ir_angle - 360.0) / 90.0, 1.5) *  (150 - min_pow);
        } else {
            // それ以外の範囲のデフォルトスピード
            speed = 150;
        }
        
        if ( 0< ir_angle && ir_angle < 180){
          move_angle = 90;
        }else{
          move_angle = 270;
        }


        MotorDrive(move_angle, speed, gryo_val);
    } else {
        // ボールが見つからないときは停止（または旋回？）
        MotorDrive(0, 0, gryo_val);
    }

    // デバッグ表示
    if (game_mode == 1){
      Serial.print("Gyro:"); Serial.print(gryo_val);
      Serial.print(" | IR_Ang:"); Serial.print(ir_angle);
      Serial.print(" | Dist:"); Serial.print(ir_dist);
      Serial.print(" | Spd:"); Serial.print(speed); // Speedも表示して確認
      Serial.print(" | Line:"); Serial.print(line_flag);
      Serial.print(" | Goal_Ang:"); Serial.print(goal_angle);
      Serial.println(); // 改行だけにする
    }
    
    //// --- ストップボタン ---
    if (digitalRead(buttonOff_Pin) == LOW) {
        if (!btn_off_holding) {
            btn_off_holding = true;
            btn_off_hold_start = millis();
        }

        unsigned long hold_duration = millis() - btn_off_hold_start;

        if (hold_duration > 2000) {
            yellow_court = !yellow_court;
            btn_off_holding = false;
            Serial.print("court changed: ");
            Serial.println(yellow_court ? "Yellow" : "Blue");

            for (int i = 0; i < 3; i++) {
                digitalWrite(LED_BUILTIN, HIGH); delay(100);  // ★ BUILTIN_LED → LED_BUILTIN
                digitalWrite(LED_BUILTIN, LOW);  delay(100);
            }

            while (digitalRead(buttonOff_Pin) == LOW) delay(10);
        }

    } else {
        if (btn_off_holding) {
            unsigned long hold_duration = millis() - btn_off_hold_start;

            if (hold_duration >= 20 && hold_duration < 2000 && game_flag == 1) {
                game_start = 0;
                game_flag = 0;
                MotorDrive(0, 0, 0);
                Serial.println("Game Stop");
            }
            btn_off_holding = false;
        }
    }

  } else { 
    // --- スタートボタン ---
    if (digitalRead(buttonOn_Pin) == LOW) {
        delay(20);
        if (digitalRead(buttonOn_Pin) == LOW) {
          game_start = 1; 
          game_flag = 0;
          Serial.println("Start Sequence...");
          while(digitalRead(buttonOn_Pin) == LOW) delay(10);
        }
    }

    // --- スタート準備 ---
    if (game_start == 1) {
      if ((gryo_val >= 60 && gryo_val <= 200) || (gryo_val <= -60 && gryo_val >= -200)) {
         MotorDrive(0, 0, 20); 
         Serial.println("Stabilizing...");
      } else {
         game_start = 0;
         game_flag = 1;
         Serial.println("GO! Game Start!");
      }
    } else {
       // 待機中
       delay(100);
    }
  }
}