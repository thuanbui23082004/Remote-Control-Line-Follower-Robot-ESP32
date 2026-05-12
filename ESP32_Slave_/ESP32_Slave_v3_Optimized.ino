// ================================================================
//  ESP32_Slave_v3_Optimized.ino
//  Dò line 2 sensor — PD Controller + Voltage Compensation
//
//  CẢI TIẾN SO VỚI v2:
//  ┌──────────────────────────────────────────────────────────────┐
//  │ 1. PD Controller     → Rẽ mượt, không văng line             │
//  │ 2. Voltage Comp.     → Tốc độ ổn định khi pin yếu/mạnh      │
//  │ 3. Soft Accel/Decel  → Không giật, không dừng đột ngột      │
//  │ 4. State Machine rõ  → Dễ debug, dễ mở rộng                 │
//  │ 5. Bỏ kickMotors()   → Không còn giật cục khó chịu          │
//  └──────────────────────────────────────────────────────────────┘
//
//  NGUYÊN LÝ PD:
//    error  = irL - irR   (-1, 0, +1)
//    dError = error - prevError
//    correction = Kp*error + Kd*dError
//    leftPWM  = BASE - correction
//    rightPWM = BASE + correction
// ================================================================

#include <Arduino.h>

// ── UART2 giao tiếp với ESP32-CAM ──────────────────────────────
#define CAM_RX 16
#define CAM_TX 17
HardwareSerial CamSerial(2);

// ── MOTOR — 2x L298N ───────────────────────────────────────────
#define ENA  25
#define IN1  26
#define IN2  27
#define ENB  13
#define IN3  14
#define IN4  21

// ── IR SENSORS (Active LOW: 0=trên line đen, 1=nền trắng) ──────
#define IR_LEFT  33
#define IR_RIGHT 32

// ── ADC đo điện áp pin (chia áp 2:1 → VCC/2 vào GPIO34) ────────
// Nối: VCC --[100kΩ]-- GPIO34 --[100kΩ]-- GND
// Nếu chưa có mạch chia áp: đặt VOLTAGE_COMP_ENABLE = false
#define VBAT_PIN            34
#define VOLTAGE_COMP_ENABLE false  // Bật lên true khi đã lắp mạch chia áp vào GPIO34
#define VBAT_NOMINAL        7.4f   // Điện áp danh định (V) — 2S LiPo
#define VBAT_MIN            6.0f   // Điện áp thấp nhất cho phép
#define VBAT_MAX            8.4f   // Điện áp đầy

// ── PWM (ESP32 LEDC) ───────────────────────────────────────────
#define PWM_FREQ       20000       // 20kHz — không nghe tiếng kêu động cơ
#define PWM_RESOLUTION 8
#define PWM_CH_ENA     4
#define PWM_CH_ENB     5

// ── MOTOR RIGHT REVERSED ───────────────────────────────────────
#define MOTOR_RIGHT_REVERSED true

// ================================================================
//  ▼▼▼  THAM SỐ — TINH CHỈNH TẠI ĐÂY  ▼▼▼
// ================================================================

// --- Tốc độ cơ bản ---
#define BASE_SPEED      160        // Manual mode
#define LINE_BASE       75         // Tốc độ trung bình khi dò line

// --- PD Controller ---
// Tăng Kp: rẽ mạnh hơn | Giảm Kp: rẽ nhẹ hơn (nhưng dễ văng)
// Tăng Kd: giảm dao động | Giảm Kd: phản ứng nhanh hơn
#define KP              38.0f      // Proportional gain
#define KD              22.0f      // Derivative gain

// --- Giới hạn tốc độ ---
#define MIN_SPEED       55         // Tốc độ tối thiểu để motor không bị chết
#define MAX_LINE_SPEED  130        // Tốc độ tối đa khi dò line

// --- Mất line ---
// Khi [1,1]: quay tìm lại theo hướng lệch cuối
#define SEARCH_SPEED    68         // Tốc độ tìm lại line
#define LOST_TIMEOUT    2000       // ms — dừng hẳn nếu không tìm lại được

// --- Tăng tốc mềm (Soft Acceleration) ---
// Tránh giật cục, bảo vệ bánh răng
#define ACCEL_STEP      8          // Mỗi chu kỳ tăng/giảm bao nhiêu PWM
#define ACCEL_INTERVAL  12         // ms giữa mỗi bước tăng tốc

// ================================================================
//  ▲▲▲  HẾT PHẦN TINH CHỈNH  ▲▲▲
// ================================================================

// ── BIẾN TOÀN CỤC ──────────────────────────────────────────────
bool  autoMode   = false;
int   manualSpeed = BASE_SPEED;
String cmdBuffer = "";

// PD state
float prevError     = 0.0f;
float prevLeft      = 0.0f;
float prevRight     = 0.0f;

// Lost-line state
int           lastTurnDir   = 0;   // -1=trái, 0=thẳng, +1=phải
unsigned long lostLineTime  = 0;

// Accel state
int   targetLeft  = 0;
int   targetRight = 0;
int   actualLeft  = 0;
int   actualRight = 0;
unsigned long lastAccelTime = 0;

// Telemetry
unsigned long lastTelemetry = 0;
const unsigned long TELE_INTERVAL = 150;

// Voltage compensation
float voltageScale = 1.0f;
unsigned long lastVbatRead = 0;
const unsigned long VBAT_INTERVAL = 500;   // Đọc pin mỗi 500ms

// ================================================================
//  VOLTAGE COMPENSATION
//  Bù điện áp: khi pin yếu → tăng PWM để giữ tốc độ không đổi
// ================================================================
void updateVoltageScale() {
#if VOLTAGE_COMP_ENABLE
  unsigned long now = millis();
  if (now - lastVbatRead < VBAT_INTERVAL) return;
  lastVbatRead = now;

  // Đọc ADC (12-bit = 0-4095, Vref = 3.3V, mạch chia áp 1:2)
  int raw = analogRead(VBAT_PIN);
  float vPin = raw * (3.3f / 4095.0f);
  float vBat = vPin * 2.0f;   // Hệ số chia áp 2:1

  // Giới hạn trong khoảng an toàn
  vBat = constrain(vBat, VBAT_MIN, VBAT_MAX);

  // Scale: ở điện áp nominal → scale=1.0; pin yếu → scale>1 (tăng PWM)
  voltageScale = VBAT_NOMINAL / vBat;
  voltageScale = constrain(voltageScale, 0.85f, 1.35f);  // Giới hạn bù ±35%
#else
  voltageScale = 1.0f;
#endif
}

// ================================================================
//  HÀM MOTOR CƠ BẢN
// ================================================================
void setLeft(bool forward) {
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);
}

void setRight(bool forward) {
  bool f = MOTOR_RIGHT_REVERSED ? !forward : forward;
  digitalWrite(IN3, f ? HIGH : LOW);
  digitalWrite(IN4, f ? LOW  : HIGH);
}

// Áp dụng voltage compensation và giới hạn MIN_SPEED
// QUAN TRỌNG: MIN_SPEED chỉ áp dụng khi speed > 0
// Nếu không, speed=0 bị đẩy lên MIN_SPEED → motor không dừng được
int applyComp(int speed) {
  if (speed <= 0) return 0;
  int s = (int)(speed * voltageScale);
  return constrain(s, MIN_SPEED, 255);
}

void writePWM(int left, int right) {
  ledcWrite(PWM_CH_ENA, constrain(left,  0, 255));
  ledcWrite(PWM_CH_ENB, constrain(right, 0, 255));
}

// ================================================================
//  SOFT ACCELERATION
//  Gọi mỗi vòng loop để dần đạt targetLeft/targetRight
// ================================================================
void updateAccel() {
  unsigned long now = millis();
  if (now - lastAccelTime < ACCEL_INTERVAL) return;
  lastAccelTime = now;

  // Left motor
  if (actualLeft < targetLeft)
    actualLeft = min(actualLeft + ACCEL_STEP, targetLeft);
  else if (actualLeft > targetLeft)
    actualLeft = max(actualLeft - ACCEL_STEP, targetLeft);

  // Right motor
  if (actualRight < targetRight)
    actualRight = min(actualRight + ACCEL_STEP, targetRight);
  else if (actualRight > targetRight)
    actualRight = max(actualRight - ACCEL_STEP, targetRight);

  writePWM(applyComp(actualLeft), applyComp(actualRight));
}

void setMotorTarget(int left, int right, bool leftFwd, bool rightFwd) {
  // Seed actual lên MIN_SPEED khi khởi động từ dừng → motor không chết lúc bắt đầu
  if (actualLeft  == 0 && left  > 0) actualLeft  = MIN_SPEED;
  if (actualRight == 0 && right > 0) actualRight = MIN_SPEED;
  setLeft(leftFwd);
  setRight(rightFwd);
  targetLeft  = constrain(left,  0, 255);
  targetRight = constrain(right, 0, 255);
}

void motorStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  writePWM(0, 0);
  targetLeft = targetRight = 0;
  actualLeft = actualRight = 0;
}

// ================================================================
//  PD LINE FOLLOWING CONTROLLER
//  Trả về correction value dựa trên error hiện tại
// ================================================================
void runPDLineFollow(int irL, int irR) {
  // error: âm = lệch trái (cần rẽ trái), dương = lệch phải
  // Sensor logic: irL=1 nghĩa là trái ra khỏi line → error âm
  float error = (float)(irR - irL);   // -1, 0, +1

  // Derivative
  float dError = error - prevError;
  prevError = error;

  // PD correction
  float correction = KP * error + KD * dError;

  // Tính tốc độ 2 bánh
  float leftSpeed  = LINE_BASE + correction;
  float rightSpeed = LINE_BASE - correction;

  // Giới hạn
  leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_LINE_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_LINE_SPEED);

  // Lưu hướng lệch để dùng khi mất line
  if (irL == 1 && irR == 0) lastTurnDir = -1;  // Lệch trái
  if (irL == 0 && irR == 1) lastTurnDir =  1;  // Lệch phải
  if (irL == 0 && irR == 0) lastTurnDir =  0;  // Thẳng

  setMotorTarget((int)leftSpeed, (int)rightSpeed, true, true);
}

// ================================================================
//  LOGIC TỰ ĐỘNG — State Machine
// ================================================================
void runAutoMode() {
  int irL = digitalRead(IR_LEFT);
  int irR = digitalRead(IR_RIGHT);
  unsigned long now = millis();

  if (irL == 0 && irR == 0) {
    // ── TRÊN LINE ── đi thẳng hoặc điều chỉnh nhẹ theo PD
    lostLineTime = 0;
    runPDLineFollow(0, 0);

  } else if (irL == 1 && irR == 0) {
    // ── LỆCH TRÁI ── cần rẽ trái
    lostLineTime = 0;
    runPDLineFollow(1, 0);

  } else if (irL == 0 && irR == 1) {
    // ── LỆCH PHẢI ── cần rẽ phải
    lostLineTime = 0;
    runPDLineFollow(0, 1);

  } else {
    // ── MẤT LINE HOÀN TOÀN [1,1] ──
    if (lostLineTime == 0) lostLineTime = now;

    if (now - lostLineTime < LOST_TIMEOUT) {
      // Quay tại chỗ theo hướng lệch cuối để tìm lại line
      if (lastTurnDir >= 0) {
        // Tìm phải: trái tiến, phải lùi
        setLeft(true); setRight(false);
      } else {
        // Tìm trái: trái lùi, phải tiến
        setLeft(false); setRight(true);
      }
      targetLeft  = SEARCH_SPEED;
      targetRight = SEARCH_SPEED;
    } else {
      // Quá thời gian → dừng hẳn
      motorStop();
    }
  }

  // Cập nhật acceleration mỗi vòng
  updateAccel();
}

// ================================================================
//  MANUAL MOTOR COMMANDS
// ================================================================
void motorForward()  { setLeft(true);  setRight(true);  }
void motorBackward() { setLeft(false); setRight(false); }
void motorLeft()     { setLeft(false); setRight(true);  }
void motorRight()    { setLeft(true);  setRight(false); }

void setManualSpeed(int speed) {
  manualSpeed = constrain(speed, 0, 255);
  targetLeft = targetRight = manualSpeed;
}

// ================================================================
//  GIAO TIẾP VỚI ESP32-CAM
// ================================================================
void processCommand(String line) {
  line.trim();
  if (!line.startsWith("CMD,")) return;

  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  if (c1 < 0 || c2 < 0) return;

  String key = line.substring(c1 + 1, c2);
  String val = line.substring(c2 + 1);
  int v = val.toInt();

  if (key == "AutoMode") {
    autoMode = (v == 1);
    if (!autoMode) {
      motorStop();
      prevError    = 0;
      lostLineTime = 0;
      lastTurnDir  = 0;
    }

  } else if (key == "MoveCar") {
    if (autoMode) return;
    setManualSpeed(manualSpeed);
    switch (v) {
      case 1: motorForward();  break;
      case 2: motorBackward(); break;
      case 3: motorLeft();     break;
      case 4: motorRight();    break;
      case 0: motorStop();     break;
    }

  } else if (key == "Speed") {
    setManualSpeed(v);

  } else if (key == "STOP") {
    autoMode = false;
    motorStop();
    prevError    = 0;
    lostLineTime = 0;
  }
}

void readFromCAM() {
  while (CamSerial.available()) {
    char c = CamSerial.read();
    if (c == '\n') {
      processCommand(cmdBuffer);
      cmdBuffer = "";
    } else {
      if (cmdBuffer.length() < 64) cmdBuffer += c;
    }
  }
}

void sendTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetry < TELE_INTERVAL) return;
  lastTelemetry = now;
  int irL = digitalRead(IR_LEFT);
  int irR = digitalRead(IR_RIGHT);
  CamSerial.printf("TEL,%d,%d\n", irL, irR);
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  CamSerial.begin(9600, SERIAL_8N1, CAM_RX, CAM_TX);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // PWM 20kHz — im lặng, động cơ ít nóng hơn 1.5kHz
  ledcSetup(PWM_CH_ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_ENB, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CH_ENA);
  ledcAttachPin(ENB, PWM_CH_ENB);

  motorStop();

  pinMode(IR_LEFT,  INPUT);
  pinMode(IR_RIGHT, INPUT);

#if VOLTAGE_COMP_ENABLE
  pinMode(VBAT_PIN, INPUT);
  analogSetAttenuation(ADC_11db);   // Cho phép đọc tới ~3.3V
  updateVoltageScale();
#endif

  delay(300);
  Serial.println("[Slave v3 Optimized] Ready — PD + VoltComp + SoftAccel");
  Serial.printf("  Kp=%.1f  Kd=%.1f  LineBase=%d  MinSpd=%d  MaxSpd=%d\n",
                KP, KD, LINE_BASE, MIN_SPEED, MAX_LINE_SPEED);
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  readFromCAM();

  if (autoMode) {
    updateVoltageScale();   // Cập nhật bù điện áp
    runAutoMode();          // Bao gồm updateAccel() bên trong
  } else {
    updateAccel();          // Vẫn chạy accel khi manual
  }

  sendTelemetry();
}
