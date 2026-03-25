#define G2_PWM 9 // Enableピン（High固定）
#define G2_DIR 7 // PWMピン（ここで制御）



// 8ビットPWM (0〜255)
const int MAX_VAL = 255;
const int STOP_VAL = 128; // 真ん中 = 停止

void setup() {
  pinMode(G2_PWM, OUTPUT);
  pinMode(G2_DIR, OUTPUT);
    // タイマー設定 (Arduino Mega前提)
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;  
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;  
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;  
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;  

  // LAP駆動設定
  digitalWrite(G2_PWM, HIGH);

  // 初期状態：停止
  analogWrite(G2_DIR, STOP_VAL);
  delay(1000);
}

void loop() {

  // -------------------------
  // 正転
  // -------------------------

  for(int i = STOP_VAL; i < MAX_VAL; i += 2)
  {
    analogWrite(G2_DIR, i);
    delay(10);
  }

  for(int i = MAX_VAL; i > STOP_VAL; i -= 2)
  {
    analogWrite(G2_DIR, i);
    delay(10);
  }

  delay(500);

  // -------------------------
  // 逆転
  // -------------------------

  for(int i = STOP_VAL; i > 0; i -= 2)
  {
    analogWrite(G2_DIR, i);
    delay(10);
  }

  for(int i = 0; i < STOP_VAL; i += 2)
  {
    analogWrite(G2_DIR, i);
    delay(10);
  }

  delay(500);
}