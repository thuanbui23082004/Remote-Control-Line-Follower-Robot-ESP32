 // ================================================================
//  ESP32_Slave_v2_NoObstacle.ino
//  Dò line 2 mức + điều khiển thủ công — ĐÃ BỎ né vật cản
//
//  NGUYÊN LÝ HOẠT ĐỘNG:
//  ┌──────────────────────────────────────────────────────────────┐
//  │  Sensor     │ Hành động                                      │
//  ├─────────────┼────────────────────────────────────────────────│
//  │ [0,0] trên  │ Đi thẳng LINE_SPEED                           │
//  │ [1,0] lệch  │ < SOFT_MS  → Quay nhẹ (1 bánh lùi nhẹ)       │
//  │   trái      │ ≥ SOFT_MS  → Quay gắt (quay tại chỗ đầy lực)  │
//  │ [0,1] lệch  │ < SOFT_MS  → Quay nhẹ (1 bánh lùi nhẹ)       │
//  │   phải      │ ≥ SOFT_MS  → Quay gắt (quay tại chỗ đầy lực)  │
//  │ [1,1] mất   │ Quay tại chỗ theo hướng lệch cuối             │
//  └──────────────────────────────────────────────────────────────┘
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

// ── IR SENSORS ─────────────────────────────────────────────────
// Active LOW: 0 = trên vạch ĐEN (line), 1 = trên NỀN TRẮNG
#define IR_LEFT  33
#define IR_RIGHT 32

// ── PWM (ESP32 LEDC) ───────────────────────────────────────────
#define PWM_FREQ       1500
#define PWM_RESOLUTION 8
#define PWM_CH_ENA     4
#define PWM_CH_ENB     5

// ── TỐCĐỘ MANUAL ───────────────────────────────────────────────
#define BASE_SPEED     160

// ================================================================
//  ▼▼▼  THAM SỐ DÒ LINE — TINH CHỈNH TẠI ĐÂY  ▼▼▼
// ================================================================

#define MIN_SPEED       50   // Giảm ngưỡng tối thiểu để xe có thể chạy chậm hơn
#define KICK_MS         5    // Kick mặc định cho chuyển trạng thái bình thường
#define KICK_MS_START   14   // Kick mạnh hơn khi vừa bật Auto dò line
#define KICK_MS_GREEN   18   // Kick mạnh hơn khi chạy lại sau đèn xanh
#define LINE_SPEED      60   // Giảm tốc độ đi thẳng (cũ 70) để xe đi từ tốn, ổn định
#define SOFT_OUTER      70   // Lực đẩy bánh ngoài khi rẽ nhẹ 
#define SOFT_INNER      0    // [QUAN TRỌNG] Đặt về 0 để bánh trong dừng hẳn, không quay lùi, tránh văng xe
#define HARD_SPEED      80   // Tăng nhẹ lực quay tại chỗ để bẻ lái dứt khoát khi lệch nặng
#define SOFT_MS         40   // Rút ngắn thời gian rẽ nhẹ (cũ 80), cho phép xe phản ứng gắt sớm hơn
#define SEARCH_SPEED    60   // Đi tìm line chậm rãi để không bị lố
#define LOST_TIMEOUT    1500

// ================================================================
//  ▲▲▲  HẾT PHẦN TINH CHỈNH  ▲▲▲
// ================================================================

// ── BIẾN TOÀN CỤC ──────────────────────────────────────────────
bool  autoMode   = false;
bool  trafficLightRed = false;   // true: RED -> stop, false: GREEN -> run
int   motorSpeed = BASE_SPEED;

String cmdBuffer = "";

unsigned long lastTelemetry  = 0;
unsigned long lostLineTime   = 0;
unsigned long turnStartTime  = 0;
int           lastTurnDir    = 0;   // -1=trái, 0=thẳng, +1=phải
int           lastState      = -1;  // 0=thẳng,1=rẽT,2=rẽP,3=tìm
int           nextKickMs     = KICK_MS;

const unsigned long TELE_INTERVAL = 150;

// ================================================================
//  HÀM MOTOR CƠ BẢN
// ================================================================

int safeSpeed(int s) { return (s <= 0) ? 0 : max(s, MIN_SPEED); }

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  ledcWrite(PWM_CH_ENA, constrain(safeSpeed(leftSpeed),  0, 255));
  ledcWrite(PWM_CH_ENB, constrain(safeSpeed(rightSpeed), 0, 255));
}

void setSpeed(int speed) {
  motorSpeed = constrain(speed, 0, 255);
  setMotorSpeeds(motorSpeed, motorSpeed);
}

#define MOTOR_RIGHT_REVERSED  true

void setLeft(bool forward) {
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);
}

void setRight(bool forward) {
  bool f = MOTOR_RIGHT_REVERSED ? !forward : forward;
  digitalWrite(IN3, f ? HIGH : LOW);
  digitalWrite(IN4, f ? LOW  : HIGH);
}

void motorForward()  { setLeft(true);  setRight(true);  }
void motorBackward() { setLeft(false); setRight(false); }

void motorLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  setRight(true);
}
void motorRight() {
  setLeft(true);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void motorSharpLeft()  { setLeft(false); setRight(true);  }
void motorSharpRight() { setLeft(true);  setRight(false); }

void motorStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(PWM_CH_ENA, 0);
  ledcWrite(PWM_CH_ENB, 0);
}

void requestKick(int ms) {
  if (ms > nextKickMs) nextKickMs = ms;
}

void kickMotors() {
  ledcWrite(PWM_CH_ENA, 255);
  ledcWrite(PWM_CH_ENB, 255);
  delay(nextKickMs);
  nextKickMs = KICK_MS;
}

// ================================================================
//  DÒ LINE — 2 MỨC PHẢN ỨNG
// ================================================================

void lineForward() {
  setLeft(true); setRight(true);
  if (lastState != 0) kickMotors();
  lastState = 0;
  setMotorSpeeds(LINE_SPEED, LINE_SPEED);
}

void lineTurnLeft(unsigned long elapsed) {
  setLeft(false); setRight(true);
  if (lastState != 1) kickMotors();
  lastState = 1;
  if (elapsed < SOFT_MS) {
    setMotorSpeeds(SOFT_INNER, SOFT_OUTER);
  } else {
    setMotorSpeeds(HARD_SPEED, HARD_SPEED);
  }
  lastTurnDir = -1;
}

void lineTurnRight(unsigned long elapsed) {
  setLeft(true); setRight(false);
  if (lastState != 2) kickMotors();
  lastState = 2;
  if (elapsed < SOFT_MS) {
    setMotorSpeeds(SOFT_OUTER, SOFT_INNER);
  } else {
    setMotorSpeeds(HARD_SPEED, HARD_SPEED);
  }
  lastTurnDir = 1;
}

void lineSearch() {
  if (lastTurnDir >= 0) {
    setLeft(true);  setRight(false);
  } else {
    setLeft(false); setRight(true);
  }
  if (lastState != 3) kickMotors();
  lastState = 3;
  setMotorSpeeds(SEARCH_SPEED, SEARCH_SPEED);
}

// ================================================================
//  LOGIC TỰ ĐỘNG
// ================================================================
void runAutoMode() {
  int irL = digitalRead(IR_LEFT);
  int irR = digitalRead(IR_RIGHT);
  unsigned long now = millis();

  if (irL == 0 && irR == 0) {
    turnStartTime = 0;
    lostLineTime  = 0;
    lastTurnDir   = 0;
    lineForward();

  } else if (irL == 1 && irR == 0) {
    lostLineTime = 0;
    if (turnStartTime == 0) turnStartTime = now;
    lineTurnLeft(now - turnStartTime);

  } else if (irL == 0 && irR == 1) {
    lostLineTime = 0;
    if (turnStartTime == 0) turnStartTime = now;
    lineTurnRight(now - turnStartTime);

  } else {
    // [1,1] Mất line hoàn toàn
    turnStartTime = 0;
    if (lostLineTime == 0) lostLineTime = now;

    if (now - lostLineTime < LOST_TIMEOUT) {
      lineSearch();
    } else {
      motorStop();
    }
  }
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
    bool prevAuto = autoMode;
    autoMode = (v == 1);
    if (!autoMode) {
      motorStop();
      lostLineTime  = 0;
      turnStartTime = 0;
      lastState     = -1;
      nextKickMs    = KICK_MS;
    } else if (!prevAuto) {
      // Lần đầu vào Auto: tăng lực khởi động để xe "bắt line" chắc hơn.
      lastState = -1;
      requestKick(KICK_MS_START);
    }

  } else if (key == "MoveCar") {
    if (autoMode) return;
    switch (v) {
      case 1: setSpeed(motorSpeed); motorForward();  break;
      case 2: setSpeed(motorSpeed); motorBackward(); break;
      case 3: setSpeed(motorSpeed); motorLeft();     break;
      case 4: setSpeed(motorSpeed); motorRight();    break;
      case 0: motorStop(); break;
    }

  } else if (key == "Speed") {
    setSpeed(v);

  } else if (key == "TrafficLight") {
    // 1 = RED (stop), 2 = GREEN (run), 0 = NONE (keep last state)
    if (v == 1) {
      trafficLightRed = true;
      motorStop();
      lastState = -1;
    } else if (v == 2) {
      bool wasRed = trafficLightRed;
      trafficLightRed = false;
      if (wasRed) {
        // Vừa nhả đèn đỏ -> đèn xanh: kick mạnh hơn để xe bứt lại.
        lastState = -1;
        requestKick(KICK_MS_GREEN);
      }
    }

  } else if (key == "STOP") {
    autoMode = false;
    motorStop();
    lostLineTime  = 0;
    turnStartTime = 0;
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
  // Bỏ distF/distL/distR, chỉ gửi IR
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

  ledcSetup(PWM_CH_ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_ENB, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CH_ENA);
  ledcAttachPin(ENB, PWM_CH_ENB);

  motorStop();
  setSpeed(BASE_SPEED);

  pinMode(IR_LEFT,  INPUT);
  pinMode(IR_RIGHT, INPUT);

  delay(300);
  Serial.println("[Slave v2 No-Obstacle] Ready — 2-level line tracking");
  Serial.printf("[Slave v2] LINE=%d SOFT_OUTER=%d SOFT_INNER=%d HARD=%d SOFT_MS=%d\n",
                LINE_SPEED, SOFT_OUTER, SOFT_INNER, HARD_SPEED, SOFT_MS);
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  readFromCAM();
  if (autoMode) {
    if (trafficLightRed) motorStop();
    else runAutoMode();
  }
  sendTelemetry();
}
