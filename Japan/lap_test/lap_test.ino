#define G2_PWM 3 // Enableピン（High固定）
#define G2_DIR 2 // PWMピン（ここで制御）

// 12ビットの最大値 (0〜4095)
const int MAX_VAL = 4095;
// LAP駆動の「停止」位置はちょうど真ん中の 2048 です
const int STOP_VAL = 2048; 

void setup() {
  pinMode(G2_PWM, OUTPUT);
  pinMode(G2_DIR, OUTPUT);

  // ★設定1: 解像度を12ビットに設定 (0〜4095)
  // 20kHzで最もきれいな波形が出せる設定です
  analogWriteResolution(12);

  // ★設定2: PWM周波数を20kHzに設定
  // LAP駆動は周波数が低いと振動するので、この20kHzが必須です
  analogWriteFrequency(G2_DIR, 20000); 

  // LAP駆動設定: 片方をHIGHに固定し、もう片方で制御します
  digitalWrite(G2_PWM, HIGH); 
  
  // 安全のため、起動時はまず「停止(50%)」を出力
  analogWrite(G2_DIR, STOP_VAL);
  delay(1000);
}

void loop() {
  // ------------------------------------------------
  // 正転動作
  // ------------------------------------------------
  
  // 停止(2048) -> 正転最大(4095) へ加速
  for(int i = STOP_VAL; i < MAX_VAL; i += 10)
  {
    analogWrite(G2_DIR, i);
    delayMicroseconds(5000); 
  }

  // 正転最大(4095) -> 停止(2048) へ減速
  for(int i = MAX_VAL; i > STOP_VAL; i -= 10)
  {
    analogWrite(G2_DIR, i);
    delayMicroseconds(5000);
  }

  delay(500); // 停止位置で少し待機

  // ------------------------------------------------
  // 逆転動作
  // ------------------------------------------------

  // 停止(2048) -> 逆転最大(0) へ加速
  for(int i = STOP_VAL; i > 0; i -= 10)
  {
    analogWrite(G2_DIR, i);
    delayMicroseconds(5000);
  }

  // 逆転最大(0) -> 停止(2048) へ減速
  for(int i = 0; i < STOP_VAL; i += 10)
  {
    analogWrite(G2_DIR, i);
    delayMicroseconds(5000);
  }
  
  delay(500); // 停止位置で少し待機
}