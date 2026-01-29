# 1 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino"
# 2 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 2
# 3 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 2
# 4 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 2
# 5 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 2



//////// GAME MODE (1:デバッグ表示あり, 0:本番用) //////////
int game_mode = 0;

// 回り込みのための計算式の係数






int move_angle = 0;

/////// ボタン関連
const int buttonOn_Pin = 52;
const int buttonOff_Pin = 53;
bool game_flag = 0; // 1: ゲーム中 (ボール追従)
int game_start = 0; // 1: スタート準備中 (姿勢制御のみ)

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
int Motor_rev[4] = {1, 1, 1, -1}; // 反転調整
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

int ir_flag = 0; // 0:ボールなし, 1:ボールあり
int ir_angle = 0; // 0〜360度
int ir_dist = 255; // 0:近い 〜 255:遠い

int line_flag = 0; // 0:ラインなし, 1:ラインあり
int line_angle = 0; // ラインの角度
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

    double mpu_degree = ypr[0] * 180 / 
# 95 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 3
                                      3.14159265358979323846 /* pi */
# 95 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino"
                                          ;
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

  pinMode(buttonOn_Pin, 0x2);
  pinMode(buttonOff_Pin, 0x2);

  // タイマー設定 (Arduino Mega前提)
  
# 117 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 117 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino"
        = (
# 117 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 3
           (*(volatile uint8_t *)(0x81)) 
# 117 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino"
                  & 0b11111000) | 0x01;
  
# 118 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 3
 (*(volatile uint8_t *)(0xB1)) 
# 118 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino"
        = (
# 118 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 3
           (*(volatile uint8_t *)(0xB1)) 
# 118 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino"
                  & 0b11111000) | 0x01;
  
# 119 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 3
 (*(volatile uint8_t *)(0x91)) 
# 119 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino"
        = (
# 119 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 3
           (*(volatile uint8_t *)(0x91)) 
# 119 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino"
                  & 0b11111000) | 0x01;
  
# 120 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 3
 (*(volatile uint8_t *)(0xA1)) 
# 120 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino"
        = (
# 120 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 3
           (*(volatile uint8_t *)(0xA1)) 
# 120 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino"
                  & 0b11111000) | 0x01;

  for(int i = 0; i < 4; i++){
    pinMode(Motor_PWM[i], 0x1);
    pinMode(Motor_DIR[i], 0x1);
    digitalWrite(Motor_PWM[i], 0x1);
    analogWrite(Motor_DIR[i], 127); // 初期停止
    Motor_rad[i] = Motor_angle[i] * 
# 127 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 3
                                   3.14159265358979323846 /* pi */ 
# 127 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino"
                                        / 180.0;
  }
    for (int i = 0; i < 4; i++) {

    digitalWrite(Motor_PWM[i], 0x1);

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
    if (pwm_val < 0) pwm_val = 0;

    digitalWrite(Motor_PWM[Motor_num], 0x1);
    analogWrite(Motor_DIR[Motor_num], pwm_val);
}

// ==========================================
// 移動関数 
// ==========================================
void MotorDrive(int face_angle, int speed_per, int gryo_val)
{
  speed_pwm = 127 * speed_per / 100;
  face_rad = face_angle * 
# 158 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino" 3
                         3.14159265358979323846 /* pi */ 
# 158 "c:\\Users\\yamas\\Documents\\GitHub\\takataka_robo_soccer_2026\\Gunma\\本番\\new_maxon_offence\\new_maxon_offence.ino"
                              / 180.0;

  for (int i = 0; i < 4; i++) {
    power[i] = (sin(Motor_rad[i] - face_rad) * speed_pwm + gryo_val) * Motor_rev[i];
  }

  double max_val = 0.0;
  for (int i = 0; i < 4; i++){
    if (((power[i])>0?(power[i]):-(power[i])) > max_val){
      max_val = ((power[i])>0?(power[i]):-(power[i]));
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
void ir_read(void) {
  Serial2.write(255);

  unsigned long request_time = micros();
  while (Serial2.available() < 5) {
    if (micros() - request_time > 10000) return;
  }

  if (Serial2.read() == 255) {
    byte buf[4];
    Serial2.readBytes(buf, 4);

    ir_flag = buf[0];
    ir_angle = buf[1] | (buf[2] << 7);
    ir_dist = buf[3] + 5;
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

    byte s0_7 = Serial1.read();
    byte s8_15 = Serial1.read();
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
// Main Loop
// ==========================================
void loop()
{
  // ジャイロ更新
  double gryo_val = getYawPitchRoll() * 0.8 ;

  // ★修正: ループの頭で speed をリセット
  speed = 0;

  if (game_flag == 1){
    line_read();
    ir_read();
    if (line_flag == 1){
      MotorDrive(line_angle + 180, 70, gryo_val);
      delay(30);
    }
    else if(ir_flag == 1){
        move_angle = 0;
        float calc_angle = ir_angle;
        //calc_angleを0から180,0から-180に設定
        if (ir_angle > 180) {
            calc_angle = ir_angle - 360;
          }

        // base(0.6)^ir_dist にすることで、距離が遠いほどほぼ直線になる
        float circ_exp = pow(pow(0.6, 1.0 / 22.0), ir_dist);
        // calc_angleを使うことで、左にあるときはマイナスの補正がかかるようになる
        float move_angle_temp = calc_angle + ((calc_angle * circ_exp * 3.5)<(-90)?(-90):((calc_angle * circ_exp * 3.5)>(90)?(90):(calc_angle * circ_exp * 3.5)));

        if (((move_angle_temp)>0?(move_angle_temp):-(move_angle_temp)) < 30) {
            speed = 150;
        } else if(ir_dist < 40){
            speed = 100;
        }else{
          speed = 60;
        }

        ///0から360に変換
        if (move_angle_temp < 0) {
            move_angle_temp = move_angle_temp + 360;
        }
        move_angle = (int)move_angle_temp;

        MotorDrive(move_angle, speed, gryo_val);
    } else {
        // ボールが見つからないときは停止（または旋回？）
        MotorDrive(0, 0, gryo_val);
    }

    // デバッグ表示
    if (game_mode == 1){
      Serial.print("Gyro:"); Serial.print(gryo_val);
      Serial.print(" | Ball_flag:"); Serial.print(ir_flag);
      Serial.print(" | IR_Ang:"); Serial.print(ir_angle);
      Serial.print(" | Dist:"); Serial.print(ir_dist);
      Serial.print(" | Spd:"); Serial.print(speed); // Speedも表示して確認
      Serial.print(" | Line:"); Serial.print(line_flag);
      Serial.print(" | move_angle:"); Serial.print(move_angle);
      Serial.println(); // 改行だけにする
    }

    // --- ストップボタン ---
    if (digitalRead(buttonOff_Pin) == 0x0) {
        delay(20);
        if (digitalRead(buttonOff_Pin) == 0x0) {
          game_start = 0;
          game_flag = 0;
          MotorDrive(0, 0, 0);
          Serial.println("Game Stop");
          while(digitalRead(buttonOff_Pin) == 0x0) delay(10);
        }
    }

  } else {
    // --- スタートボタン ---
    if (digitalRead(buttonOn_Pin) == 0x0) {
        delay(20);
        if (digitalRead(buttonOn_Pin) == 0x0) {
          game_start = 1;
          game_flag = 0;
          Serial.println("Start Sequence...");
          while(digitalRead(buttonOn_Pin) == 0x0) delay(10);
        }
    }

    // --- スタート準備 ---
    if (game_start == 1) {
      if ((gryo_val >= 60 && gryo_val <= 200) || (gryo_val <= -60 && gryo_val >= -200)) {
         MotorDrive(0, 0, 40);
         Serial.println("Stabilizing...");
      } else {
         game_start = 0;
         game_flag = 1;
         Serial.println("GO! Game Start!");
      }
    } else {
       // 待機中
       delay(100);
       Serial.println(gryo_val);
    }
  }
}
