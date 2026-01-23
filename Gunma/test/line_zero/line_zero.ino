// ピン定義 (Seeed XIAO RP2040)
const int PIN_MUX_S0 = D4;
const int PIN_MUX_S1 = D5;
const int PIN_MUX_S2 = D10;
const int PIN_MUX_S3 = D9;
const int PIN_SIG1   = A1; 
const int PIN_SIG2   = A0; 

const int CALIB_COUNT = 100; // 計測回数

// 合計値を保存するため、intではなくlongを使います（オーバーフロー防止）
long sum[32]; 
int avg[32];

void setup() {
  Serial.begin(115200);

  // シリアルモニタが開くまで待機（必須）
  while (!Serial) delay(10);

  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);
  
  pinMode(PIN_SIG1, INPUT);
  pinMode(PIN_SIG2, INPUT);

  // 本番環境と同じ解像度に設定
  analogReadResolution(12);

  // 変数初期化
  for (int i = 0; i < 32; i++) sum[i] = 0;

  Serial.println("=========================================");
  Serial.println("   Line Sensor Calibration Start");
  Serial.println("   Place the robot on the BLACK field.");
  Serial.println("   Do NOT place on the white line.");
  Serial.println("=========================================");
  delay(2000); // 配置のための猶予時間
  Serial.println("Measuring...");

  // --- 100回 計測ループ ---
  for (int cnt = 0; cnt < CALIB_COUNT; cnt++) {
    for (int i = 0; i < 16; i++) {
      // MUX切り替え
      digitalWrite(PIN_MUX_S0, (i >> 0) & 1);
      digitalWrite(PIN_MUX_S1, (i >> 1) & 1);
      digitalWrite(PIN_MUX_S2, (i >> 2) & 1);
      digitalWrite(PIN_MUX_S3, (i >> 3) & 1);

      delayMicroseconds(50); // 安定待ち

      // 値を加算 (SIG1: 0-15番, SIG2: 16-31番)
      sum[i]      += analogRead(PIN_SIG1);
      sum[i + 16] += analogRead(PIN_SIG2);
    }
    delay(5); // 次の計測まで少し待つ
  }

  // --- 平均値の計算 ---
  for (int i = 0; i < 32; i++) {
    avg[i] = sum[i] / CALIB_COUNT;
  }

  // --- 結果の表示（コピペ用フォーマット） ---
  Serial.println("Done!");
  Serial.println();
  Serial.println("// ↓↓↓ ここからコピー ↓↓↓");
  Serial.println();
  
  Serial.println("int line_debug[32] = {");
  
  // 見やすく整形して出力
  for (int i = 0; i < 32; i++) {
    Serial.print("  ");
    Serial.print(avg[i]);
    
    // 最後の要素以外にはカンマをつける
    if (i < 31) {
      Serial.print(",");
    }
    
    // 8個ごとに改行して見やすくする
    if ((i + 1) % 8 == 0) {
      Serial.println();
    }
  }
  
  Serial.println("};");
  Serial.println();
  Serial.println("// ↑↑↑ ここまでコピー ↑↑↑");
}

void loop() {
  // 計測が終わったら何もしない
}