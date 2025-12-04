// DSR-1502 ラインセンサ (A0ピンのみ使用)

const int LINE_PIN = A0;  // センサーをつなぐピン
int threshold = 100;      // ラインのしきい値（ここを調整)

void setup() {
  Serial.begin(9600); // シリアル通信の開始
}

void loop() {
  int val = analogRead(LINE_PIN);

  Serial.print("Value: ");
  Serial.print(val);

  if (val > threshold) {
    Serial.println("  ---> [ LINE!! ]"); // ライン検知！
  } else {
    Serial.println("       ( ... )");     // ラインなし
  }

  delay(200); // 読みやすくするために少し待つ
}