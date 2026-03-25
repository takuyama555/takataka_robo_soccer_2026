#include <math.h> 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define gyro_p 0.8 

//////// GAME MODE (1:デバッグ表示あり, 0:本番用) //////////
int game_mode = 0;

// 回り込みのための計算式の係数
#define CIRC_BASE pow(0.6, 1.0 / 20.0)
#define CIRC_WEIGHT 3.5

#define STRAIGHT_SPEED 60
#define CIRC_SPEED     50

/////// ボタン関連 
const int buttonOn_Pin = 20;   // BUTTON1
const int buttonOff_Pin = 21;  // BUTTON2
bool game_flag = 0;            
int game_start = 0;            

// タイムアウト設定 (ms)
const unsigned long TIMEOUT_MS = 5;

// --- 変数定義 ---
double print_data[32]; 

// ★モータピン 
int Motor_DIR[4] = {2,5,8,9};
int Motor_PWM[4] = {3,4,9,8};
int speed_pwm = 0; 
double face_rad = 0.0; 
int Motor_angle[4] = {45, 135, 225, 315};
double Motor_rad[4] = {0.0, 0.0, 0.0, 0.0};
double Motor_rev[4] = {1, -1, -1, 1}; 
double power[4] = {0.0, 0.0, 0.0, 0.0};
int speed = 0; 

int recent_line_angle[10] = {999,999,999,999,999,999,999,999,999,999};
int line_time = 0;

bool yellow_flag = 0;
int  yellow_angle = 0;
bool blue_flag   = 0;
int  blue_angle   = 0;
int goal_angle = 0;

/// diffence用の変数
int move_angle = 0;
float diff = 0.0;
float diff_ratio = 0.0;
float speed_ratio = 0.0;
float min_speed = 20.0; 
float max_speed = 90.0;

/////// ir関連 ///////

// --- 三角関数テーブル ---
float cos_table[32];
float sin_table[32];

const int PIN_MUX_S0 = 26; 
const int PIN_MUX_S1 = 27; 
const int PIN_MUX_S2 = 28; 
const int PIN_MUX_S3 = 29; 
const int PIN_SIG1   = 24; 
const int PIN_SIG2   = 25; 

// --- 設定 ---
const int MAX_ADC_VAL = 4095; 
const int SENSOR_THRESHOLD = 500; 

// --- グローバル変数 ---
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

bool escaping_line = false; // 現在ライン回避中かどうか
int escape_angle = 0;       // 回避するために進むべき方向
unsigned long line_lost_time = 0; // ラインを見失った時刻

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

  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);
  
  pinMode(PIN_SIG1, INPUT);
  pinMode(PIN_SIG2, INPUT);

  analogReadResolution(12);  // 0-4095
  analogWriteResolution(12); // 0-4095でPWM出力

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
    power[i] = (sin(Motor_rad[i] - face_rad) * speed_pwm + gyro_val*10) * Motor_rev[i];  
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

    // 読み取り (0-15番)
    analogRead(PIN_SIG1); 
    int raw1 = analogRead(PIN_SIG1);
    ir_values[i] = MAX_ADC_VAL - raw1;
    if(ir_values[i] < 0) ir_values[i] = 0;

    // 読み取り (16-31番)
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

  if (MAX_value > SENSOR_THRESHOLD){
    ball_flag = 1;
  } else {
    ball_flag = 0;
  }
}

// ==========================================
// 角度計算
// ==========================================
void calc_angle(){

  // ---  全センサー計算 (ALL) ---
  sum_x_all = 0.0;
  sum_y_all = 0.0;

  for (int i = 0; i < 32; i++){
      sum_x_all += cos_table[i] * ir_values[i]; // ★テーブル参照に変更
      sum_y_all += sin_table[i] * ir_values[i]; // ★テーブル参照に変更
  }

  // 角度
  float rad_all = atan2(sum_y_all, sum_x_all);
  if (rad_all < 0) rad_all += M_PI * 2;
  ir_deg_all = rad_all * 180.0 / M_PI;

  ////  検証の結果すべてのセンサーを使う距離計算はうまくいかなかったため、削除 ////


  // --- 部分センサー計算 (Part) ---
  sum_x_part = 0.0;
  sum_y_part = 0.0;

  // MAX_pinを中心に前後計算
  for (int k = -6; k <= 6; k++){
      int idx = (MAX_pin + k + 32) % 32; // リングバッファ

      sum_x_part += cos_table[idx] * ir_values[idx]; // ★テーブル参照に変更
      sum_y_part += sin_table[idx] * ir_values[idx]; // ★テーブル参照に変更
  }
  
  // 角度
  float rad_part = atan2(sum_y_part, sum_x_part);
  if (rad_part < 0) rad_part += M_PI * 2;
  ir_deg_part = rad_part * 180.0 / M_PI;

  // 距離 (安全対策)
  double mag_sq_part = sum_x_part * sum_x_part + sum_y_part * sum_y_part;
  if (mag_sq_part > 1.0) {
    ir_dist_part = log(mag_sq_part); 
  } else {
    ir_dist_part = 0;
  }
  if (ir_dist_part > 20.6) {
    ir_dist_part = 20.6;
  }
  ir_dist_part = (20.6 - ir_dist_part) * 100;
}

void get_ball_angle(){
  sensor_read();

  if (ball_flag == 1){ 
    calc_angle();
  } else {
    ir_deg_all = 0;
    ir_dist_all = 0;
    ir_deg_part = 0;
    ir_dist_part = 0;
  }

  if ( ir_dist_part > 40){
    ball_angle = ir_deg_part;
  }else{
    ball_angle = ir_deg_all;
  }
  // 角度から15度引いてズレを直す
  if (ball_angle < 45 || ball_angle > 315)
  ball_angle -= 15;
  
  // もし引いた結果がマイナスになったら360を足して0〜359度の範囲に戻す
  if (ball_angle < 0) {
      ball_angle += 360;
  } else if (ball_angle >= 360) {
      ball_angle -= 360;
  }
  ball_dist = ir_dist_part;
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
    Serial3.write(253); 

    uint32_t startTime = millis();
    while (Serial3.available() < 7) {
        if (millis() - startTime > 5) return; 
    }

    uint8_t header = Serial3.read();
    
    if (header == 253) { 
        yellow_flag = (Serial3.read() == 1);
        uint8_t y_low = Serial3.read();
        uint8_t y_high = Serial3.read();
        yellow_angle = y_low | (y_high << 7);

        blue_flag = (Serial3.read() == 1);
        uint8_t b_low = Serial3.read();
        uint8_t b_high = Serial3.read();
        blue_angle = b_low | (b_high << 7);
        
        goal_angle = 0; 

        if (yellow_flag && blue_flag) {
            int y_diff = min(yellow_angle, abs(180 - yellow_angle));
            if (yellow_angle > 180) y_diff = min(y_diff, 360 - yellow_angle);

            int b_diff = min(blue_angle, abs(180 - blue_angle));
            if (blue_angle > 180) b_diff = min(b_diff, 360 - blue_angle);

            if (y_diff <= b_diff) {
                goal_angle = yellow_angle;
            } else {
                goal_angle = blue_angle;
            }
        } else if (yellow_flag) {
            goal_angle = yellow_angle;
        } else if (blue_flag) {
            goal_angle = blue_angle;
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
    get_ball_angle(); 
    camera_read();

    // --- ライン処理 ---
    if (line_flag == 1) {
        if (135 < line_angle && line_angle < 225){
          if (ball_angle < 180) {
            escape_angle = 45;
          }
          else if(ball_angle > 180){
            escape_angle = 315;
          }
        }
        // 新しくラインを踏んだ、あるいは踏み続けている場合
        else if (!escaping_line) {
            escaping_line = true;
            // 踏んだ瞬間の逆ベクトルを逃げる方向に設定
            escape_angle = line_angle + 180; 
            if (escape_angle >= 360) escape_angle -= 360;
        }
        line_lost_time = millis(); // 踏んでいる間はタイマーを更新
        
        MotorDrive(escape_angle, 100, gyro_val); // 決まった方向に全力で離れる
        delay(3); 
    } 
    else if (escaping_line) {
        // センサーは消えたが、まだ「回避モード」中の場合
        // 完全に離脱したと言い切れるまで（例：100ms間）は逃げ続ける
        if (millis() - line_lost_time < 75) { 
            MotorDrive(escape_angle, 100, gyro_val);
        } else {
            escaping_line = false; // 十分離れたので回避終了
        }
    }
    // --- ボール処理 ---
    else if(ball_flag == 1){
        min_speed = 20.0;
        max_speed = 90.0;
        if (0 < ball_angle && ball_angle < 180) {
            move_angle = 100; 
            
            // 正面(0度)からのズレを計算。90度以上離れたら最大スピードで固定する
            diff = ball_angle;
            if (diff > 90) diff = 90; 

            // 0.0 〜 1.0 の比率を計算
            diff_ratio = diff / 90.0;
            speed_ratio = diff_ratio * diff_ratio;
            
            // 最低スピードから最大スピードの間で2次関数カーブを作る
            speed = min_speed + (max_speed - min_speed) * speed_ratio;
        }
        else if(180 < ball_angle && ball_angle < 340) {
            move_angle = 260;
            
            // 正面(360度)からのズレを計算。90度以上は最大スピードで固定
            diff = 360 - ball_angle;
            if (diff > 90) diff = 90;

            diff_ratio = diff / 90.0;
            speed_ratio = diff_ratio * diff_ratio;
            
            speed = min_speed + (max_speed - min_speed) * speed_ratio;
        } else {
            // ボールが真正面(0度) または 340〜360度の不感帯にあるとき
            speed = 0;
        }

        MotorDrive(move_angle, speed, gyro_val);
    } else {
        MotorDrive(0, 0, gyro_val);
    }

    // --- デバッグ表示 ---
    if (game_mode == 1){
      Serial.print("Gyro:"); Serial.print(gyro_val);
      Serial.print(" | IR_Ang:"); Serial.print(ball_angle);
      Serial.print(" | Dist:"); Serial.print(ball_dist);
      Serial.print(" | Spd:"); Serial.print(speed); 
      Serial.print(" | Line:"); Serial.print(line_flag);
      Serial.print(" | Goal_Ang:"); Serial.print(goal_angle);
      Serial.println(); 
    }
    
    // --- ストップボタン ---
    if (digitalRead(buttonOff_Pin) == LOW) {
        delay(20); 
        if (digitalRead(buttonOff_Pin) == LOW) {
          game_start = 0;
          game_flag = 0;
          MotorDrive(0, 0, 0); 
          Serial.println("Game Stop");
          while(digitalRead(buttonOff_Pin) == LOW) delay(10);
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