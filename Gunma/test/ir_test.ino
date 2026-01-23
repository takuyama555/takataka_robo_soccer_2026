// ==========================================
// ピン設定
// ==========================================
// Arduino Uno/Nano等はDを付けずに数値のみで指定
#include <math.h>
const int COM_PIN[2] = {A0, A1};      
const int MUX_PIN[4] = {4, 5, 9, 10}; // S0, S1, S2, S3

const int IR_NUM = 16;        // 片方のMUXにつながるセンサーの数
const int IR_VAL_MAX = 1023;  // 最大値 (10bit)

int ir_values[32]; // 値を保存する配列
float ir_angle[32];
float ir_x[32];
float ir_y[32];
float BallAngle = 0.0;
float ball_dist = 0.0;

void setBallAngle();
void printRow(const char* label, int startIdx, int endIdx);
void printFormattedValue(int val);

void setup() {
    Serial.begin(115200);
    pinMode(COM_PIN[0], INPUT);
    pinMode(COM_PIN[1], INPUT);
    for (int i = 0; i < 4; i++) {
        pinMode(MUX_PIN[i], OUTPUT);
    }
    Serial.println("--- IR Sensor Monitor (Inverted) ---");
    Serial.println("補正: (1023 - 読み取り値) を表示します");
    delay(1000);
    for (int i = 0; i <32 ; i++){
      ir_angle[i] = 11.25 * i * M_PI / 180  ;
    }
}

void loop() {
    // --- 1. 全センサー読み取り ---
    for (int i = 0; i < IR_NUM; i++) {
        // マルチプレクサ切り替え
        for (int j = 0; j < 4; j++) {
            digitalWrite(MUX_PIN[j], (i >> j) & 0x01);
        }
        delayMicroseconds(50); // 安定待ち

        // ■■■ ここで反転補正を入れています ■■■
        // 生の値が小さいときに「近い」と判定されるセンサーの場合、
        // 1023から引くことで「値が大きい＝近い」に変換できます。
        int raw0 = analogRead(COM_PIN[0]);
        int raw1 = analogRead(COM_PIN[1]);

        ir_values[i]      = IR_VAL_MAX - raw0;
        ir_values[i + 16] = IR_VAL_MAX - raw1;
    }
    setBallAngle();
    // --- 2. 一覧表示 ---
    printRow("0-7 : ", 0, 8);
    printRow("8-15: ", 8, 16);
    printRow("16-23 : ", 16, 24);
    printRow("24-31 : ", 24, 32);
    Serial.print("BallAngle [deg] : ");
    Serial.println(BallAngle * 180.0 / M_PI, 2);
    Serial.print("Ball_Distant : ");
    Serial.println(ball_dist);

    Serial.println("--------------------------------------------------"); 
    delay(200); 
}

// 行表示用ヘルパー関数
void printRow(const char* label, int startIdx, int endIdx) {
    Serial.print(label);
    for (int i = startIdx; i < endIdx; i++) {
        printFormattedValue(ir_values[i]);
    }
    Serial.println();
}

// 3桁表示整形
void printFormattedValue(int val) {
    Serial.print("[");
    if (val < 10) Serial.print("  ");
    else if (val < 100) Serial.print(" ");
    Serial.print(val);
    Serial.print("] ");
}

void setBallAngle(){
    float sum_x = 0;
    float sum_y = 0;
    for (int i = 0; i < 32; i++) {
        sum_x += cos(ir_angle[i]) * ir_values[i];
        sum_y += sin(ir_angle[i]) * ir_values[i];
    }

    BallAngle = atan2(sum_y, sum_x);

    // 0～2π に変換
    if (BallAngle < 0) {
        BallAngle += 2 * M_PI;
    }
    ball_dist = sqrt(sum_x * sum_x + sum_y * sum_y);

}
