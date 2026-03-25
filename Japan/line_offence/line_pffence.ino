// ピン定義 (Seeed XIAO RP2040)
const int PIN_MUX_S0 = D4;
const int PIN_MUX_S1 = D5;
const int PIN_MUX_S2 = D10;
const int PIN_MUX_S3 = D9;
const int PIN_SIG1   = A1; 
const int PIN_SIG2   = A0; 

float line_deg = 0.0;
float line_rad = 0.0;
int line = 0; // ライン検知フラグ
float sum_x = 0;
float sum_y = 0;
int line_flag[32];

// ★修正1: valuesをグローバル変数にして、どこからでも読めるように移動
int values[32]; 

int line_debug[32] = {
  254,  312,  208,  319,  311,  248,  239,  329,
  173,  223,  331,  238,  380,  249,  289,  393,
  574,  376,  382,  448,  246,  271,  241,  308,
  302,  354,  382,  391,  240,  312,  268,  313
};
// 設定値
const int MAX_ADC_VAL = 1025; 

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  
  // ★追加: PCと繋がるまで待機（デバッグ表示を見逃さないため）
  // 不要ならコメントアウトしてください
  // while (!Serial) delay(10); 

  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);
  
  pinMode(PIN_SIG1, INPUT);
  pinMode(PIN_SIG2, INPUT);

  analogReadResolution(12);
}

void sensor_read(){
  sum_x = 0;
  sum_y = 0;

  // --- センサー読み取り ---
  for (int i = 0; i < 16; i++) {
    digitalWrite(PIN_MUX_S0, (i >> 0) & 1);
    digitalWrite(PIN_MUX_S1, (i >> 1) & 1);
    digitalWrite(PIN_MUX_S2, (i >> 2) & 1);
    digitalWrite(PIN_MUX_S3, (i >> 3) & 1);

    delayMicroseconds(50); 

    // 読み取り (PIN_SIG1: 0~15)
    values[i] = analogRead(PIN_SIG1) - line_debug[i];
    if(values[i] < 0) values[i] = 0;

    // 読み取り (PIN_SIG2: 16~31)
    // ★修正2: line_debugの添字を [i] から [i+16] に修正
    values[i + 16] = analogRead(PIN_SIG2) - line_debug[i + 16]; 
    if(values[i + 16] < 0) values[i + 16] = 0;
  } 

  // --- 閾値判定 ---
  line = 0; 
  values[1] = 0;
  for (int i=0; i<32; i++){
    line_flag[i] = 0;
    if (values[i] > 500){
      line_flag[i] = 1;
      line = 1; 
    }
  }

  // --- 角度計算---
  if (line == 1) { 
      for (int i = 0; i < 32; i++){
          sum_x = sum_x + cos(11.25 * i * PI / 180.0) * line_flag[i];
          sum_y = sum_y + sin(11.25 * i * PI / 180.0) * line_flag[i];
      }
      line_rad = atan2(sum_y, sum_x);
      if (line_rad < 0) line_rad = line_rad + PI * 2;
      line_deg = line_rad * 180.0 / PI;
  }
}

// メインマイコンにデータを送信する関数
void uart_send(void) {
    byte buf[7]; 
    
    buf[0] = (byte)line; 
    
    uint16_t tmp_deg = (uint16_t)line_deg;
    buf[1] = tmp_deg & 0x7F;       
    buf[2] = (tmp_deg >> 7) & 0x7F; 

    buf[3] = 0; buf[4] = 0; buf[5] = 0; buf[6] = 0;

    for (int i = 0; i < 8; i++) {
        if (line_flag[i])      buf[3] |= (1 << i); 
        if (line_flag[i+8])    buf[4] |= (1 << i); 
        if (line_flag[i+16])   buf[5] |= (1 << i); 
        if (line_flag[i+24])   buf[6] |= (1 << i); 
    }
    
    Serial1.write(254);     
    Serial1.write(buf, 7);  
}

void loop() {
  sensor_read();
  
  // リクエスト受信処理
  if (Serial1.available()) {            
        uint8_t header = Serial1.read();  
        if (header == 254) {             
            uart_send();                  
        }
        while(Serial1.available()) Serial1.read();
    }

  // // ★修正3: 全センサーの値を表示
  // //CSV形式（カンマ区切り）で出力
  Serial.print("Sensors:");
  for (int i = 0; i < 32; i++) {
    Serial.print(values[i]);
    if (i < 31) Serial.print(","); // 最後の要素以外はカンマを入れる
  }
  Serial.println(); // 改行
  Serial.println(line_deg);

  // delay(50); // 表示が見えるように少しウェイトを入れる
}