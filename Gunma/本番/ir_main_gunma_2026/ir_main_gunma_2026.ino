#include <Arduino.h>
#include <math.h>

// --- ピン定義 (Seeed XIAO RP2040) ---
const int PIN_MUX_S0 = D4;
const int PIN_MUX_S1 = D5;
const int PIN_MUX_S2 = D9;
const int PIN_MUX_S3 = D10;
const int PIN_SIG1   = A0; 
const int PIN_SIG2   = A1; 

// --- 設定 ---
const int MAX_ADC_VAL = 4072; 
const int SENSOR_THRESHOLD = 500; // ボール有無の閾値 (環境に合わせて調整)

// --- グローバル変数 ---
int values[32]; 
int ball_flag = 0;

float ir_deg_all = 0.0;
float ir_dist_all = 0.0;

int MAX_value = 0;
int MAX_pin = 0;

float ir_deg_part = 0.0;
float ir_dist_part = 0.0;

// ベクトル計算用変数 (loop内で毎回生成するよりグローバルの方が速い場合があるため)
float sum_x_all = 0;
float sum_y_all = 0;
float sum_x_part = 0;
float sum_y_part = 0;
float ball_angle = 0;

void setup() {
  Serial.begin(115200);  // デバッグ用
  Serial1.begin(115200); // 通信用
  
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);
  
  pinMode(PIN_SIG1, INPUT);
  pinMode(PIN_SIG2, INPUT);

  analogReadResolution(12); // 0-4095
}

void sensor_read(){
  // --- リセット ---
  ball_flag = 0;
  MAX_value = 0;
  MAX_pin = 0;

  // --- センサー読み取り ---
  for (int i = 0; i < 16; i++) {
    // MUX切り替え
    digitalWrite(PIN_MUX_S0, (i >> 0) & 1);
    digitalWrite(PIN_MUX_S1, (i >> 1) & 1);
    digitalWrite(PIN_MUX_S2, (i >> 2) & 1);
    digitalWrite(PIN_MUX_S3, (i >> 3) & 1);

    delayMicroseconds(20); // 少し待つ (50は長すぎるかもなので20に短縮)

    // 読み取り (0-15番)
    // 1回空読みすると値が安定しますが、速度優先なら1回でも可
    analogRead(PIN_SIG1); 
    int raw1 = analogRead(PIN_SIG1);
    values[i] = MAX_ADC_VAL - raw1;
    if(values[i] < 0) values[i] = 0;

    // 読み取り (16-31番)
    analogRead(PIN_SIG2);
    int raw2 = analogRead(PIN_SIG2);
    values[i + 16] = MAX_ADC_VAL - raw2; 
    if(values[i + 16] < 0) values[i + 16] = 0;
  } 

  // --- 最大値とボール有無の判定 ---
  for (int i = 0; i < 32; i++){
    // 最大値更新
    if (values[i] > MAX_value){
      MAX_value = values[i];
      MAX_pin = i;
    }
  }

  // ノイズ対策：最大値が閾値を超えているか？
  if (MAX_value > SENSOR_THRESHOLD){
    ball_flag = 1;
  } else {
    ball_flag = 0;
  }
}

void calc_angle(){
  // --- 1. 全センサー計算 (ALL) ---
  sum_x_all = 0.0;
  sum_y_all = 0.0;

  for (int i = 0; i < 32; i++){
      sum_x_all += cos(11.25 * i * PI / 180.0) * values[i];
      sum_y_all += sin(11.25 * i * PI / 180.0) * values[i];
  }

  // 角度
  float rad_all = atan2(sum_y_all, sum_x_all);
  if (rad_all < 0) rad_all += PI * 2;
  ir_deg_all = rad_all * 180.0 / PI;

  ////  検証の結果すべてのセンサーを使う距離計算はうまくいかなかったため、削除 ////

  //  距離 (対数計算の安全対策)
  // double mag_sq_all = sum_x_all * sum_x_all + sum_y_all * sum_y_all;
  // if (mag_sq_all > 1.0) {
  //   ir_dist_all = log(mag_sq_all) ; 
  // } else {
  //   ir_dist_all = 0;
  // }
  //   if (ir_dist_all > 18.3) ir_dist_all = 18.3;
  //  ir_dist_all = (18.3 - ir_dist_all) * 15;


  // --- 2. 部分センサー計算 (Part) ---
  sum_x_part = 0.0;
  sum_y_part = 0.0;

  // MAX_pinを中心に前後計算
  for (int k = -6; k <= 6; k++){
      int idx = (MAX_pin + k + 32) % 32; // リングバッファ

      sum_x_part += cos(11.25 * idx * PI / 180.0) * values[idx];
      sum_y_part += sin(11.25 * idx * PI / 180.0) * values[idx];
  }
  
  // 角度
  float rad_part = atan2(sum_y_part, sum_x_part);
  if (rad_part < 0) rad_part += PI * 2;
  ir_deg_part = rad_part * 180.0 / PI;

  // 距離 (安全対策)
  double mag_sq_part = sum_x_part * sum_x_part + sum_y_part * sum_y_part;
  if (mag_sq_part > 1.0) {
    ir_dist_part = log(mag_sq_part) ; 
  } else {
    ir_dist_part = 0;
  }
  if (ir_dist_part > 20.6) {
    ir_dist_part = 20.6;}
  ir_dist_part = (20.6 - ir_dist_part) * 100;
}
  

// メインマイコンにデータを送信する関数
void uart_send(void) {
    byte buf[4];
    
    buf[0] = ball_flag; // 0 or 1
    
    // 角度送信 (0~360度)
    uint16_t tmp_deg = (uint16_t)(ball_angle + 0.5f);
    if (tmp_deg >= 360) tmp_deg = 359;
    buf[1] = tmp_deg & 0x7F;       // 下位7bit
    buf[2] = (tmp_deg >> 7) & 0x7F; // 上位ビット (念のためマスク)
    
    // 距離送信 (0~254に制限)
    int dist_temp = (int)ir_dist_part;
    if (dist_temp > 254) dist_temp = 254;
    if (dist_temp < 0) dist_temp = 0;
    buf[3] = (byte)dist_temp;
    
    Serial1.write(255);     // ヘッダー
    Serial1.write(buf, 4);  // データ
}

void loop() {
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
    ball_angle = ir_deg_all;
  }else{
    ball_angle = ir_deg_all;
  }
  
  // リクエスト受信処理
  if (Serial1.available()) {            
        uint8_t header = Serial1.read();  
        if (header == 255) {              
            uart_send();                  
        }
        // 受信バッファのゴミ掃除
        while(Serial1.available()) Serial1.read();
    }

 // --- 結果の表示 (USBシリアル側) ---
  // Serial.print("MAX_Pin: ");
  // Serial.print(MAX_pin);
  // Serial.print(" | ball_flag: ");
  // Serial.print(ball_flag);
  
  // // 小数点1桁まで表示
  // Serial.print(" | All_deg: ");
  // Serial.print(ir_deg_all, 1); 
  // Serial.print(" | Part_deg: ");
  // Serial.print(ir_deg_part, 1);


  // Serial.print(" | All_dist: ");
  // Serial.print(ir_dist_all); 
  // Serial.print(" | Part_dist: ");
  // Serial.println(ir_dist_part);

  //  delay(100); // 不要なら削除可
}