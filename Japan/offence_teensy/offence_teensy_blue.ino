#include <math.h> 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define gyro_p 0.8 

//////// GAME MODE (1:デバッグ表示あり, 0:本番用) //////////
int game_mode = 1;

/////// どのゴールを認識するかの設定 ////////
bool yellow_court = false;  // true = Yellow, false = Blue

#define CIRC_BASE pow(0.6, 1.0 / 20.0)
#define CIRC_WEIGHT 3.5

#define STRAIGHT_SPEED 60
#define CIRC_SPEED     50

/////// ボタン関連 
const int buttonOn_Pin = 20;
const int buttonOff_Pin = 21;
bool game_flag = 0;            
int game_start = 0;            

// タイムアウト設定 (ms)
const unsigned long TIMEOUT_MS = 5;

// --- 変数定義 ---
double print_data[32]; 

// ★モータピン 
int Motor_DIR[4] = {2,5,8,6};
int Motor_PWM[4] = {3,4,9,7};
int speed_pwm = 0; 
double face_rad = 0.0; 
int Motor_angle[4] = {45, 135, 225, 315};
double Motor_rad[4] = {0.0, 0.0, 0.0, 0.0};
double Motor_rev[4] = {-1, -1, 1, -1};  
double power[4] = {0.0, 0.0, 0.0, 0.0};
int speed = 0; 
int move_angle = 0;

int recent_line_angle[10] = {999,999,999,999,999,999,999,999,999,999};
int line_time = 0;

bool yellow_flag = 0;
int  yellow_angle = 0;
bool blue_flag   = 0;
int  blue_angle   = 0;
int goal_angle = 0;
float move_angle_temp = 0;
int yellow_height = 0;
int blue_height   = 0;
int goal_height   = 0;

int ir_pin_1 = 39;
int ir_pin_2 = 38; 
int ir1 = 0;
int ir2 = 0;


bool btn_off_holding   = false;
unsigned long btn_off_hold_start = 0;

/////// ir関連 ///////
float cos_table[32];
float sin_table[32];

const int PIN_MUX_S0 = 26; 
const int PIN_MUX_S1 = 27; 
const int PIN_MUX_S2 = 28; 
const int PIN_MUX_S3 = 29; 
const int PIN_SIG1   = 24; 
const int PIN_SIG2   = 25; 

const int MAX_ADC_VAL = 4095; 
const int SENSOR_THRESHOLD = 500; 

int ir_values[32]; 
int ball_flag = 0;

float ir_deg_all = 0.0;
float ir_dist_all = 0.0;

int MAX_value = 0;
int MAX_pin = 0;

float ir_deg_part = 0.0;
float ir_dist_part = 0.0;

float sum_x_all = 0;
float sum_y_all = 0;
float sum_x_part = 0;
float sum_y_part = 0;
float ball_angle = 0;
float ball_dist = 0;
int hold_flag = 0;   

// --- ジャイロ変数 ---
MPU6050 mpu;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
double current_yaw = 0.0; 

/// ---ライン関係---
int ir_flag = 0;   
int ir_angle = 0;  
int ir_dist = 255; 

bool escaping_line = false;
int escape_angle = 0;
unsigned long line_lost_time = 0;

int line_flag = 0;   
int line_angle = 0;  
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
    print_data[18] = mpu_degree;
    current_yaw = mpu_degree; 
  }
  return current_yaw; 
}

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200); 
  Serial4.begin(115200); 
  
  setupMPU();
  
  pinMode(buttonOn_Pin, INPUT_PULLUP);
  pinMode(buttonOff_Pin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);  // ★ BUILTIN_LED → LED_BUILTIN

  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);
  
  pinMode(PIN_SIG1, INPUT);
  pinMode(PIN_SIG2, INPUT);

  analogReadResolution(12);
  analogWriteResolution(12);

  for(int i = 0; i < 4; i++){
    pinMode(Motor_PWM[i], OUTPUT);
    pinMode(Motor_DIR[i], OUTPUT);
    analogWriteFrequency(Motor_DIR[i], 20000); 

    digitalWrite(Motor_PWM[i], HIGH);
    analogWrite(Motor_DIR[i], 2048); 
    
    Motor_rad[i] = Motor_angle[i] * M_PI / 180.0;
  }

  for(int i = 0; i < 32; i++){
    cos_table[i] = cos(11.25 * i * M_PI / 180.0);
    sin_table[i] = sin(11.25 * i * M_PI / 180.0);
  }
}

// ==========================================
// モーター出力
// ==========================================
void Motor(int Motor_num, int motor_speed) {
    int pwm_val = 2048 + motor_speed;
    if (pwm_val > 4095) pwm_val = 4095;
    if (pwm_val < 0)   pwm_val = 0;

    digitalWrite(Motor_PWM[Motor_num], HIGH); 
    analogWrite(Motor_DIR[Motor_num], pwm_val); 
}

// ==========================================
// 移動関数 
// ==========================================
void MotorDrive(int face_angle, int speed_per, int gyro_val) {
  speed_pwm = 2048 * speed_per / 100; 
  face_rad = face_angle * M_PI / 180.0; 

  for (int i = 0; i < 4; i++) {
    power[i] = (sin(Motor_rad[i] - face_rad) * speed_pwm + gyro_val*30) * Motor_rev[i];  
  }

  double max_val = 0.0;
  for (int i = 0; i < 4; i++){
    if (abs(power[i]) > max_val){
      max_val = abs(power[i]);
    }
  }

  double ratio = 1.0;
  if (max_val > 2048.0){
    ratio = 2048.0 / max_val;
  }

  for(int i = 0; i < 4; i++){
    int final_power = (int)(power[i] * ratio);
    Motor(i, final_power);
  }
}

// ==========================================
// アナログマルチプレクサ IR読み取り
// ==========================================
void sensor_read(){
  ball_flag = 0;
  MAX_value = 0;
  MAX_pin = 0;

  for (int i = 0; i < 16; i++) {
    digitalWriteFast(PIN_MUX_S0, (i >> 0) & 1); 
    digitalWriteFast(PIN_MUX_S1, (i >> 1) & 1);
    digitalWriteFast(PIN_MUX_S2, (i >> 2) & 1);
    digitalWriteFast(PIN_MUX_S3, (i >> 3) & 1);

    delayMicroseconds(20); 

    analogRead(PIN_SIG1); 
    int raw1 = analogRead(PIN_SIG1);
    ir_values[i] = MAX_ADC_VAL - raw1;
    if(ir_values[i] < 0) ir_values[i] = 0;

    analogRead(PIN_SIG2); 
    int raw2 = analogRead(PIN_SIG2);
    ir_values[i + 16] = MAX_ADC_VAL - raw2; 
    if(ir_values[i + 16] < 0) ir_values[i + 16] = 0;
  } 

  for (int i = 0; i < 32; i++){
    if (ir_values[i] > MAX_value){
      MAX_value = ir_values[i];
      MAX_pin = i;
    }
  }

  ball_flag = (MAX_value > SENSOR_THRESHOLD) ? 1 : 0;
}

// ==========================================
// 角度計算
// ==========================================
void calc_angle(){
  sum_x_all = 0.0;
  sum_y_all = 0.0;

  for (int i = 0; i < 32; i++){
      sum_x_all += cos_table[i] * ir_values[i];
      sum_y_all += sin_table[i] * ir_values[i];
  }

  float rad_all = atan2(sum_y_all, sum_x_all);
  if (rad_all < 0) rad_all += M_PI * 2;
  ir_deg_all = rad_all * 180.0 / M_PI;

  sum_x_part = 0.0;
  sum_y_part = 0.0;

  for (int k = -6; k <= 6; k++){
      int idx = (MAX_pin + k + 32) % 32;
      sum_x_part += cos_table[idx] * ir_values[idx];
      sum_y_part += sin_table[idx] * ir_values[idx];
  }
  
  float rad_part = atan2(sum_y_part, sum_x_part);
  if (rad_part < 0) rad_part += M_PI * 2;
  ir_deg_part = rad_part * 180.0 / M_PI;

  double mag_sq_part = sum_x_part * sum_x_part + sum_y_part * sum_y_part;
  if (mag_sq_part > 1.0) {
    ir_dist_part = log(mag_sq_part); 
  } else {
    ir_dist_part = 0;
  }
  if (ir_dist_part > 20.63) {
    ir_dist_part = 20.63;
  }
  ir_dist_part = (20.63 - ir_dist_part) * 100;
}

void get_ball_info(){
  sensor_read();

  if (ball_flag == 1){ 
    calc_angle();
  } else {
    ir_deg_all = 0;
    ir_dist_all = 0;
    ir_deg_part = 0;
    ir_dist_part = 0;
  }

  ball_angle = ir_deg_all;
  ball_dist = ir_dist_part;
  hold_flag = 0;

  // ボール保持判定
  ir1 = analogRead(ir_pin_1);
  ir2 = analogRead(ir_pin_2);
  if (ir1 < 750 || ir2 < 750) {
     hold_flag = 1;
  }
}

// ==========================================
// ライン取得 
// ==========================================
void line_read(){
  while (Serial4.available()) Serial4.read();
  
  Serial4.write(254); 

  unsigned long start_time = millis();
  while (Serial4.available() < 8) {
    if (millis() - start_time > TIMEOUT_MS) return;
  }

  if (Serial4.read() == 254) { 
    byte r_flag = Serial4.read();       
    byte r_ang_low = Serial4.read();    
    byte r_ang_high = Serial4.read();   
    
    byte s0_7   = Serial4.read();       
    byte s8_15  = Serial4.read();       
    byte s16_23 = Serial4.read();       
    byte s24_31 = Serial4.read();       

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
      line_check[sensor_num] = (sensor_raw[i] >> bit) & 1;
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
void loop() {
  double gyro_val = getYawPitchRoll() * gyro_p ; 
  speed = 0; 

  if (game_flag == 1){
    line_read();
    get_ball_info(); 
    camera_read();

    // --- ライン処理 ---
    if (line_flag == 1) {
        if (!escaping_line) {
            escaping_line = true;
            escape_angle = line_angle + 180; 
            if (escape_angle >= 360) escape_angle -= 360;
        }
        line_lost_time = millis();
        
        MotorDrive(escape_angle, 100, gyro_val);
        delay(3); 
    } 
    else if (escaping_line) {
        if (millis() - line_lost_time < 100) { 
            MotorDrive(escape_angle, 100, gyro_val);
        } else {
            escaping_line = false;
        }
    }
    // --- ボール処理 ---
    else if(ball_flag == 1){
        line_time = 0;
        for (int i = 0; i < 10; i++){
          recent_line_angle[i] = 999;
        }

        float calc_ang = ball_angle; 
        float move_angle_temp = 0;
        if (ball_angle > 180) {
          calc_ang = ball_angle - 360; 
        }

        if (hold_flag == 1){
          move_angle_temp = goal_angle;
          speed = 90;
        } else if(ball_dist > 50) {
          move_angle_temp = ball_angle;
          speed = 90;
        }
        else{
          float circ_exp = pow(CIRC_BASE, ball_dist * 2.0);
          move_angle_temp = calc_ang + constrain(calc_ang * circ_exp * CIRC_WEIGHT, -90, 90);
          speed = 90;
        }
        if (move_angle_temp < 0) {
            move_angle_temp = move_angle_temp + 360;
        }

        move_angle = (int)move_angle_temp;
        MotorDrive(move_angle, speed, gyro_val);
      }else {
        MotorDrive(0, 0, gyro_val);
    }

    // --- デバッグ表示 ---
    if (game_mode == 1){
      Serial.print("Gyro:"); Serial.print(gyro_val);
      Serial.print(" | IR_Ang:"); Serial.print(ball_angle);
      Serial.print(" | Dist:"); Serial.print(ball_dist);
      Serial.print(" | Spd:"); Serial.print(speed); 
      Serial.print(" | Line:"); Serial.print(line_flag);
      Serial.print(" | IR_dist:"); Serial.print(ball_dist);
      Serial.print(" | Goal_Ang:"); Serial.print(goal_angle);
      Serial.print(" | Goal_H:"); Serial.print(goal_height);  // ★ 高さも表示
      Serial.print(" | move_Ang:"); Serial.print(move_angle);
      Serial.print(" | ir1:"); Serial.print(ir1);
      Serial.print(" | ir2:"); Serial.print(ir2);
      Serial.print(" | hold:"); Serial.print(hold_flag);
      Serial.println(); 
    }
    
    // --- ストップボタン ---
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

    // --- スタート準備 (姿勢制御) ---
    if (game_start == 1) {
      if ((gyro_val >= 60 && gyro_val <= 200) || (gyro_val <= -60 && gyro_val >= -200)) {
         MotorDrive(0, 0, 20); 
         Serial.println("Stabilizing...");
      } else {
         game_start = 0;
         game_flag = 1;
         Serial.println("GO! Game Start!");
         
      }
    } else {
       delay(100);
    }
  }
}