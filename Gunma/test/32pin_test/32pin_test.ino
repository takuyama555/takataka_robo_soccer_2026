// ピン定義 (Seeed XIAO RP2040)
const int PIN_MUX_S0 = D4;
const int PIN_MUX_S1 = D5;
const int PIN_MUX_S2 = D9;
const int PIN_MUX_S3 = D10;
const int PIN_SIG1   = A0; // 0-15番
const int PIN_SIG2   = A1; // 16-31番

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);
  
  pinMode(PIN_SIG1, INPUT);
  pinMode(PIN_SIG2, INPUT);

  // RP2040のアナログ読み取りを12bit(0-4095)に設定
  analogReadResolution(12);
}

void loop() {
  int values[32]; // 値を保存する箱

  // 0〜15のカウントで、MUX1とMUX2を同時に切り替える
  for (int i = 0; i < 16; i++) {
    // 1. ピンの切り替え (ビット演算)
    digitalWrite(PIN_MUX_S0, (i >> 0) & 1);
    digitalWrite(PIN_MUX_S1, (i >> 1) & 1);
    digitalWrite(PIN_MUX_S2, (i >> 2) & 1);
    digitalWrite(PIN_MUX_S3, (i >> 3) & 1);

    // ★重要: 切り替え後の待機時間 (これがないと隣の値が混ざります)
    delayMicroseconds(50); 

    // 2. 読み取り (16-32番: A0ピン)
    analogRead(PIN_SIG1); // 1回空読み (リフレッシュ)
    int raw1 = analogRead(PIN_SIG1);
    values[i] = 4072 - raw1;
    if(values[i] < 0) values[i] = 0;

    // 3. 読み取り (1-15番: A1ピン)
    analogRead(PIN_SIG2); // 1回空読み
    int raw2 = analogRead(PIN_SIG2);
    values[i + 16] = 4072 - raw2; 
    if(values[i + 16] < 0) values[i + 16] = 0;
  }

  // --- 結果の表示 ---
  Serial.println("\n--- Sensor Values (0-31) ---");
  
  // 0-15番を表示
  Serial.print("00-15: ");
  for (int i = 0; i < 16; i++) {
    Serial.print(values[i]);
    Serial.print("\t"); // タブ区切り
  }
  Serial.println("");

  // 16-31番を表示
  Serial.print("16-31: ");
  for (int i = 16; i < 32; i++) {
    Serial.print(values[i]);
    Serial.print("\t");
  }
  Serial.println("");

  delay(200); // 読みやすくするために少し待つ
}