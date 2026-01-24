#include <math.h> 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define gryo_p 0.8 

//////// GAME MODE (1:デバッグ表示あり, 0:本番用) //////////
int game_mode = 1;

//////// 初期位置戻るよう (カメラないため)   ///////////
int x_position = 1; 

/////// ボタン関連
const int buttonOn_Pin = 52;   
const int buttonOff_Pin = 53;  
int game_flag = 0;            // 1: ゲーム中 (ボール追従)
int game_start = 0;            // 1: スタート準備中 (姿勢制御のみ)

// タイムアウト設定 (ms)
const unsigned long TIMEOUT_MS = 15;

// --- 変数定義 ---
double print[32];
int Motor_PWM[4] = {4,6,3,7};
int Motor_DIR[4] = {2,8,5,9};
int speed_pwm = 0; 
double face_rad = 0.0; 
int Motor_angle[4] = {45, 135, 225, 315};
double Motor_rad[4] = {0.0, 0.0, 0.0, 0.0};
int Motor_rev[4] = {1, 1, -1, -1}; // 反転調整
double power[4] = {0.0, 0.0, 0.0, 0.0};

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

  TCCR1B = (TCCR1B & 0b11111000) | 0x01;  // Timer1
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;  // Timer2
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;  // Timer3
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;  // Timer4

  for(int i = 0; i < 4; i++){
    pinMode(Motor_PWM[i], OUTPUT);
    pinMode(Motor_DIR[i], OUTPUT);
    digitalWrite(Motor_PWM[i], HIGH);
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
void Motor(int Motor_num, int speed)
{
    int pwm_val = 127 + speed;
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
  Serial2.write(255); // リクエスト

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

  // 32個のセンサーを展開
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

float get_line_x() {
  float temp_sum_x = 0;
  int sensor_count_x = 0;
  for (int i = 0; i < 32; i++) {
    if (line_check[i] == 1) {
       float rad = 11.25 * i * PI / 180.0;
       temp_sum_x += cos(rad);
       sensor_count_x++;
    }
  }
  if (sensor_count_x > 0) return temp_sum_x / sensor_count_x; 
  else return 0;
}

float get_line_y() {
  float temp_sum_y = 0;
  int sensor_count_y = 0;
  for (int i = 0; i < 32; i++) {
    if (line_check[i] == 1) {
       float rad = 11.25 * i * PI / 180.0;
       temp_sum_y += sin(rad);
       sensor_count_y ++;
    }
  }
  if (sensor_count_y > 0) return temp_sum_y / sensor_count_y; 
  else return 0;
}

// ==========================================
// キーパー動作用関数
// ==========================================
void line_trace(float gryo_val){
  
  float line_power = 3.0; 

  // --- X軸の計算 ---
  float l_x_tmp = get_line_x(); 
  int sign_l_x = 1;
  if (l_x_tmp < 0) sign_l_x = -1;
  float l_x_curved = pow(abs(l_x_tmp), line_power) * sign_l_x;

  // --- Y軸の計算 ---
  float l_y_tmp = get_line_y();
  int sign_l_y = 1;
  if (l_y_tmp < 0) sign_l_y = -1;
  float l_y_curved = pow(abs(l_y_tmp), line_power) * sign_l_y;

  // ゲイン設定
  float line_gain = 5.0; 

  float l_x = l_x_curved * line_gain;
  float l_y = l_y_curved * line_gain;

  float ball_coefficient = 10.0;   // coefficientに修正
  float ir_rad = ir_angle * PI / 180.0; 
  float b_y = sin(ir_rad) * ball_coefficient;
  
  float m_x = l_x;
  float m_y = l_y + b_y;
  
  // 移動角度・速度計算
  float move_rad = atan2(m_y , m_x);
  float move_angle = move_rad * 180.0 / PI; // float計算推奨

  float speed_coefficient = 7.0;
  float move_speed = sqrt(m_x * m_x + m_y * m_y) * speed_coefficient;
  move_speed = constrain(move_speed, 0, 100);


  MotorDrive((int)move_angle, (int)move_speed, (int)gryo_val);
}


// ==========================================
// Main Loop
// ==========================================
void loop()
{
  double gryo_val = getYawPitchRoll() * gryo_p ;

  if (game_flag != 0){
    line_read();
    ir_read(); 
    if (line_flag == 0){
      game_flag = 1;
    }

    ////// 初期位置に戻るとき //////
    if (game_flag == 1){
      if (line_flag == 1){
        if (70 < line_angle && line_angle < 110){
          x_position = 1;
        }else if (250 < line_angle && line_angle < 290){
          x_position = 2;
        }else {
          game_flag = 2;
        }
      }

      if (x_position == 1){
        MotorDrive(190, 30, (int)gryo_val);
      }
      if (x_position == 2){
        MotorDrive(170, 30, (int)gryo_val);
      }
    }
    
    ////// 初期位置にたどり着き、キーパー状態の時  ///////
    if (game_flag == 2){
      // 左右に振られすぎた場合の復帰ロジック(x_position更新)
      if (line_flag == 1 && 205 < line_angle && line_angle < 245){
        x_position = 2;
      }
      if (line_flag == 1 && 115 < line_angle && line_angle < 155){
        x_position = 1;
      }
      
      line_trace(gryo_val);
    }

    // デバッグ表示
    if (game_mode == 1){
      Serial.print("Gyro:"); Serial.print(gryo_val);
      Serial.print(" | IR_Ang:"); Serial.print(ir_angle);
      Serial.print(" | line_Ang:"); Serial.print(line_angle);
      Serial.print("| game_flag:"); Serial.println(game_flag);
       delay(50); 
    }
    
    // --- ストップボタン処理 ---
    if (digitalRead(buttonOff_Pin) == LOW) {
        delay(20); 
        if (digitalRead(buttonOff_Pin) == LOW) {
          game_start = 0;
          game_flag = 0;
          MotorDrive(0, 0, 0); // 完全停止
          Serial.println("Game Stop");
          while(digitalRead(buttonOff_Pin) == LOW) delay(10);
        }
    }

  } else { 
    // ---  スタートボタン処理 ---
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
         Serial.println(gryo_val);
        delay(10); // 少し待つ
    }
  }
}
