#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

// --- 制御用変数 ---
bool dmpReady = false;  // DMP初期化成功フラグ
uint8_t devStatus;      // デバイス状態
uint16_t packetSize;    // FIFOパケットサイズ
uint8_t fifoBuffer[64]; // FIFOバッファ

// --- 姿勢データ用変数 ---
Quaternion q;           // 四元数
VectorFloat gravity;    // 重力ベクトル
float ypr[3];           // [yaw, pitch, roll]
double current_yaw = 0;
double print_data[20];  // print配列（変数名が予約語と被らないよう少し変更しました）

// ボタンピンの定義
const int buttonOn_Pin = 2;
const int buttonOff_Pin = 3;

// ==========================================
// ジャイロ初期化
// ==========================================
void setupMPU() {
  Serial.println("Step 1: Wire.begin()...");
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock
  
  Serial.println("Step 2: mpu.initialize()...");
  mpu.initialize();

  Serial.println("Step 3: mpu.testConnection()...");
  if (!mpu.testConnection()) {
    Serial.println("ERROR: MPU6050 connection failed! 配線を確認してください。");
    return;
  }
  Serial.println("MPU6050 connection successful.");

  Serial.println("Step 4: mpu.dmpInitialize()...");
  devStatus = mpu.dmpInitialize();

  // ★あなたのロボット固有の値
  mpu.setXAccelOffset(-1811);
  mpu.setYAccelOffset(-1108);
  mpu.setZAccelOffset(960);
  mpu.setXGyroOffset(72);
  mpu.setYGyroOffset(-9);
  mpu.setZGyroOffset(0);

  if (devStatus == 0) {
    Serial.println("Step 5: Calibrating...");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    
    Serial.println("Step 6: Enabling DMP...");
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("SUCCESS: DMP ready!");
  } else {
    Serial.print("ERROR: DMP Initialization failed. Code: ");
    Serial.println(devStatus);
  }
}

// ==========================================
// ジャイロ値取得
// ==========================================
double getYawPitchRoll() {
  if (!dmpReady) return 0; // DMPの準備ができていなければ0を返す

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // ラジアンから度に変換
    double mpu_degree = ypr[0] * 180 / M_PI;
    print_data[18] = mpu_degree;
    current_yaw = mpu_degree; 
  }
  return current_yaw; 
}

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  // ★重要: Teensyの起動が早すぎるため、シリアルモニタが開くまで少し待つ
  while (!Serial && millis() < 4000); 
  
  Serial.println("\n--- System Start ---");

  pinMode(buttonOn_Pin, INPUT_PULLUP);
  pinMode(buttonOff_Pin, INPUT_PULLUP);

  // ジャイロのセットアップ処理を呼び出し
  setupMPU();
}

// ==========================================
// Loop
// ==========================================
void loop() {
  // DMPが準備できていない場合はループ処理をスキップ
  if (!dmpReady) {
    delay(500);
    return;
  }

  // ジャイロ値の取得と更新
  double yaw = getYawPitchRoll();
  
  // 値を確認（Teensyが速すぎるので、少し間引いて表示）
  static uint32_t lastPrintTime = 0;
  if (millis() - lastPrintTime > 100) { // 100ミリ秒に1回だけ表示
    Serial.print("Yaw: ");
    Serial.println(yaw);
    lastPrintTime = millis();
  }
}