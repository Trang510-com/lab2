#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <PID_v1.h>

// ===== PINOUT (ESP32) =====
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 13
#define ENA 25
#define ENB 33
#define pinR_A 16
#define pinR_B 32
#define pinL_A 4
#define pinL_B 35

#define RXD2 15
#define TXD2 23

// ===== PWM CONFIG =====
#define PWM_FREQ 1500
#define PWM_RES 8

// ===== MPU =====
MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// ===== SETPOINT GỐC (Điểm đứng thẳng lý thuyết của xe) =====
#define BASE_SETPOINT (180.0-0.52)

// ===== PID VARIABLES =====
double input, output, setpoint;
double yinput, youtput, ysetpoint;
double vInput, vOutput, vSetpoint; 

// ===== ENCODER & POSITION =====
volatile long countL = 0;
volatile long countR = 0;
double robotPosition = 0;

// ===== KHAI BÁO CẤU HÌNH PID CHUẨN XÁC =====
// CHÚ Ý 1: Đổi sang REVERSE để bánh xe đuổi theo hướng ngã đỡ xe dậy
PID pid(&input, &output, &setpoint, 23.0, 0.0, 1.5, DIRECT);

// CHÚ Ý 2: Hướng Xoay (Yaw) đặt DIRECT để bù lực xoay chéo bánh xe
PID rot(&yinput, &youtput, &ysetpoint, 3.5, 0.0, 0.1, DIRECT);

// CHÚ Ý 3: Vận tốc đặt DIRECT để khi vInput dương (xe trôi tiến), vOutput sẽ tăng dương, 
// kéo setpoint góc tăng lên bắt xe phải ngửa người ra sau để phanh lại.
PID vel(&vInput, &vOutput, &vSetpoint, 0.15, 0.007, 0.00, DIRECT);

// ===== MOTOR =====
int MotorA, MotorB;

// ===== BLUETOOTH =====
HardwareSerial blue(2);

// ===== ENCODER INTERRUPTS =====
void IRAM_ATTR encoderL() {
  if (digitalRead(pinL_A) == digitalRead(pinL_B)) {
    countL++;
  } else {
    countL--;
  }
}

void IRAM_ATTR encoderR() {
  if (digitalRead(pinR_A) == digitalRead(pinR_B)) {
    countR++;
  } else {
    countR--;
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(1000); 

  // KHỞI TẠO MPU
  Wire.begin(21, 22);
  Wire.setClock(100000); 
  Wire.setTimeOut(50);   

  mpu.initialize();
  mpu.reset();
  delay(200);
  mpu.initialize();
  delay(100);

  if (!mpu.testConnection()) {
    Serial.println(F("❌ Kết nối MPU6050 THẤT BẠI!"));
    while (1);
  }
  Serial.println(F("✅ MPU6050 OK!"));

  blue.begin(9600, SERIAL_8N1, RXD2, TXD2);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  Serial.println(F("Đang tải cấu hình DMP..."));
  uint8_t devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(237);
  mpu.setYAccelOffset(-1403);
  mpu.setZAccelOffset(1075);
  mpu.setXGyroOffset(20);
  mpu.setYGyroOffset(-44);
  mpu.setZGyroOffset(-39);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Wire.setClock(400000); 
    Serial.println(F("✅ MPU6050 và DMP đã sẵn sàng!"));
  } else {
    Serial.print(F("❌ Khởi động DMP lỗi: "));
    Serial.println(devStatus);
    while (1);
  }

  // Khởi tạo trạng thái ban đầu của các biến điều khiển
  setpoint  = BASE_SETPOINT;
  vSetpoint = 0;

  // Cấu hình PID Thăng bằng
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255); // Khớp độ rộng phát xung 8-bit chân ga ga của cầu H

  // Cấu hình PID Xoay
  rot.SetMode(AUTOMATIC);
  rot.SetSampleTime(10);
  rot.SetOutputLimits(-60, 60);

  // Cấu hình PID Vận tốc
  vel.SetMode(AUTOMATIC);
  vel.SetSampleTime(40); // PID vận tốc đáp ứng tần suất thấp hơn (40ms) để lọc nhiễu cơ học bánh xe
  vel.SetOutputLimits(-20.0, 20.0); // CHÚ Ý: Mở rộng biên độ lệch góc phanh từ 3.0 lên 20.0 độ!

  // Encoder pins
  pinMode(pinL_B, INPUT);
  pinMode(pinR_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinL_A), encoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinR_A), encoderR, RISING);
  
  // Khóa hướng ngẫu nhiên ban đầu của xe làm đích Yaw (Tránh quay chong chóng)
  delay(500);
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ysetpoint = ypr[0] * 180.0 / PI; 
  }
}

// ===== LOOP =====
void loop() {
  if (!dmpReady) return;

  // Đọc góc từ DMP liên tục thời gian thực
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    input  = ypr[1] * 180.0 / PI + 180.0;
    yinput = ypr[0] * 180.0 / PI;
  }

  // ===== ĐỌC TỐC ĐỘ ENCODER (AN TOÀN NẮT) =====
  static long lastCountL = 0;
  static long lastCountR = 0;
  static unsigned long lastTime = 0;

  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;

  if (dt >= 0.01) {
    noInterrupts();
    long snapL = countL;
    long snapR = countR;
    interrupts();

    long deltaL = snapL - lastCountL;
    long deltaR = snapR - lastCountR;

    double newVel = ((deltaL + deltaR) / 2.0) / dt;
    vInput = 0.7 * vInput + 0.3 * newVel; // Bộ lọc thông thấp giảm chấn nhiễu răng cưa

    robotPosition += (deltaL + deltaR) / 2.0;
    //vInput = robotPosition;

    lastCountL = snapL;
    lastCountR = snapR;
    lastTime = now;
  }

  // ===== TÍNH TOÁN CASCADE PID LỒNG NHAU =====
  vel.Compute();
  setpoint = BASE_SETPOINT - vOutput; // Phanh/Bù góc động từ vận tốc dạt sang vòng thăng bằng

  pid.Compute(); // Tính toán lực thăng bằng tức thời
  rot.Compute(); // Tính toán lực khóa hướng lái

  // CHÚ Ý: Loại bỏ lọc Kalman gây trễ công suất ở đây, xả trực tiếp Output của PID ra motor
  MotorA = output + youtput; // Lực bánh trái
  MotorB = output - youtput; // Lực bánh phải

  setMotor(MotorA, MotorB);

  // ===== ĐIỀU KHIỂN QUA BLUETOOTH =====
  if (blue.available()) {
    char c = blue.read();

    if (c == 'f') vSetpoint = -150; // Tạo vận tốc đích âm $\rightarrow$ Xe ngả nhẹ người ra trước tiến lên
    if (c == 'b') vSetpoint = 150;  // Tạo vận tốc đích dương $\rightarrow$ Xe ngả người lùi lại
    if (c == 'l') ysetpoint -= 8;   // Xoay nhẹ hướng đích sang trái
    if (c == 'r') ysetpoint += 8;   // Xoay nhẹ hướng đích sang phải
    if (c == 's') {                 
      vSetpoint = 0;
      ysetpoint = yinput;           // Khóa hướng ngay tại vị trí hiện tại lúc bấm dừng
    }
  }

  // ===== SERIAL MONITOR DEBUG =====
  static unsigned long printTime = 0;
  if (millis() - printTime > 50) { // Giới hạn tốc độ in 50ms để không làm nghẹt chip
    Serial.print(input);         Serial.print("\t");
    Serial.print(setpoint);      Serial.print("\t");
    Serial.print(output);        Serial.print("\t");
    Serial.print(vInput);        Serial.print("\t");
    Serial.print(vOutput);        Serial.print("\t");
    Serial.println(robotPosition);
    printTime = millis();
  }
}

// ===== ĐIỀU KHIỂN ĐỘNG CƠ CÔNG SUẤT =====
void setMotor(int left, int right) {
  // Đồng bộ với dải OutputLimits mới (-255 đến 255)
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  // LEFT MOTOR
  if (left > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    left = -left;
  }

  // RIGHT MOTOR
  if (right > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    right = -right;
  }

  ledcWrite(ENA, (uint8_t)left);
  ledcWrite(ENB, (uint8_t)right);
}