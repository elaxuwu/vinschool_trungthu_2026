#include <Arduino.h>

// --- Cấu hình chân Arduino Uno ---
constexpr uint8_t BUTTON_PIN = 2;
constexpr uint8_t AUTO_LED_PIN = 3;
constexpr uint8_t MANUAL_LED_PIN = 4;

// Cấu hình L298N - Động cơ DC JGA25 280RPM
// Chuyển sang D10 (PWM) và D11 (PWM)
constexpr uint8_t IN1_PIN = 10;
constexpr uint8_t IN2_PIN = 11;

// Cấu hình Công tắc hành trình (Limit Switch - Safe Stop)
// Module 3 chân: VCC -> 5V, GND -> GND, OUT -> D5 (Tiến/Thuận) & D6 (Lùi/Nghịch)
constexpr uint8_t LIMIT_FWD_PIN = 5; // Công tắc giới hạn đầu thuận / tiến
constexpr uint8_t LIMIT_REV_PIN = 6; // Công tắc giới hạn đầu nghịch / lùi
// Hầu hết module công tắc hành trình 3 chân có LED: khi chạm sẽ kéo chân OUT xuống LOW.
// (Đổi thành HIGH nếu module của bạn bình thường LOW, khi bấm ra HIGH)
constexpr int LIMIT_ACTIVE_LEVEL = LOW;

constexpr int CENTER = 511;
constexpr int DEAD_ZONE = 80;
constexpr int JOY_RANGE = 380;   // Chỉ cần gạt ~75% hành trình là đã đạt kịch trần MAX_SPEED (255)
constexpr int MIN_SPEED = 60;   // PWM tối thiểu để động cơ JGA25 thắng ma sát hộp số
constexpr int MAX_SPEED = 255;  // PWM tối đa (0 - 255)
constexpr int MOTOR_DIRECTION = 1; // Đổi thành -1 nếu muốn đảo chiều quay thực tế
constexpr unsigned long DEBOUNCE_MS = 40;
constexpr unsigned long RAMP_INTERVAL_MS = 10; // Tần suất cập nhật gia tốc (ms)
constexpr int RAMP_STEP = 35;                 // Bước tăng/giảm PWM (đạt max 255 trong ~70ms)

bool autoMode = false;
bool manualReady = false;
int lastReading = HIGH;
int stableButton = HIGH;
unsigned long changedAt = 0;

int currentSpeed = 0;
int autoSpeed = 0;
unsigned long lastRampAt = 0;
unsigned long autoStartedAt = 0;
unsigned long autoDuration = 500;

bool buttonPressed(int reading, unsigned long now) {
  if (reading != lastReading) {
    lastReading = reading;
    changedAt = now;
  }
  if (now - changedAt >= DEBOUNCE_MS && reading != stableButton) {
    stableButton = reading;
    return stableButton == LOW;
  }
  return false;
}

int joystickSpeed(int y) {
  int diff = y - CENTER;
  if (abs(diff) <= DEAD_ZONE) return 0;

  if (diff > 0) {
    long val = map(diff, DEAD_ZONE, JOY_RANGE, MIN_SPEED, MAX_SPEED);
    return (int)constrain(val, MIN_SPEED, MAX_SPEED);
  } else {
    long val = map(-diff, DEAD_ZONE, JOY_RANGE, MIN_SPEED, MAX_SPEED);
    return -(int)constrain(val, MIN_SPEED, MAX_SPEED);
  }
}

// --- Cấu hình và Hàm Debug Joystick ---
constexpr bool DEBUG_JOYSTICK = false;            // Đổi thành false nếu muốn tắt in debug
constexpr unsigned long DEBUG_INTERVAL_MS = 200; // In mỗi 200ms (5 lần/s), không nghẽn chip
int rawY_min = 1023;
int rawY_max = 0;
unsigned long lastDebugAt = 0;

void debugJoystick(int rawY, int targetSpeed, bool ready, bool hitFwd, bool hitRev, unsigned long now) {
  if (!DEBUG_JOYSTICK) return;

  // Tự động ghi nhớ khoảng dao động thực tế khi gạt cần
  if (rawY < rawY_min) rawY_min = rawY;
  if (rawY > rawY_max) rawY_max = rawY;

  // Giới hạn tần suất in Serial để không nghẽn Arduino và không lag động cơ
  if (now - lastDebugAt >= DEBUG_INTERVAL_MS) {
    lastDebugAt = now;
    int diff = rawY - CENTER;

    Serial.print(F("[JOY] ADC: "));
    Serial.print(rawY);
    Serial.print(F(" (Min: "));
    Serial.print(rawY_min);
    Serial.print(F(", Max: "));
    Serial.print(rawY_max);
    Serial.print(F(") | Diff: "));
    if (diff >= 0) Serial.print('+');
    Serial.print(diff);
    Serial.print(F(" | PWM Target: "));
    Serial.print(targetSpeed);
    Serial.print(F(" | PWM Out: "));
    Serial.print(currentSpeed);
    Serial.print(F(" | Status: "));
    if (!ready) {
      Serial.print(F("WAIT_CENTER"));
    } else if (targetSpeed == 0) {
      Serial.print(F("STOP"));
    } else if (targetSpeed > 0) {
      Serial.print(F("FWD"));
    } else {
      Serial.print(F("REV"));
    }
    if (hitFwd) Serial.print(F(" | [!] LIMIT_FWD HIT"));
    if (hitRev) Serial.print(F(" | [!] LIMIT_REV HIT"));
    Serial.println();
  }
}

bool isLimitFwdHit() {
  return digitalRead(LIMIT_FWD_PIN) == LIMIT_ACTIVE_LEVEL;
}

bool isLimitRevHit() {
  return digitalRead(LIMIT_REV_PIN) == LIMIT_ACTIVE_LEVEL;
}

void releaseMotor() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  currentSpeed = 0;
}

void setMotorPWM(int speed) {
  speed = speed * MOTOR_DIRECTION;

  if (speed == 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    return;
  }

  int pwm = constrain(abs(speed), 0, 255);

  if (speed > 0) {
    // Quay thuận: IN1 = LOW, IN2 = PWM
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  } else {
    // Quay nghịch: IN1 = PWM, IN2 = LOW (D10 và D11 đều có PWM phần cứng trên Arduino Uno)
    analogWrite(IN1_PIN, pwm);
    digitalWrite(IN2_PIN, LOW);
  }
}

void showMode() {
  digitalWrite(MANUAL_LED_PIN, autoMode ? LOW : HIGH);
  digitalWrite(AUTO_LED_PIN, autoMode ? HIGH : LOW);
  Serial.println(autoMode ? F("MODE=AUTO") : F("MODE=MANUAL"));
}

void runMotor(int targetSpeed, unsigned long now) {
  // Lớp bảo vệ Safe Stop cấp thấp: Khóa ngay nếu đang muốn chạy về phía công tắc bị chạm
  if (targetSpeed * MOTOR_DIRECTION > 0 && isLimitFwdHit()) {
    targetSpeed = 0;
    currentSpeed = 0;
  }
  if (targetSpeed * MOTOR_DIRECTION < 0 && isLimitRevHit()) {
    targetSpeed = 0;
    currentSpeed = 0;
  }

  // Tăng tốc phản hồi tức thì nhưng vẫn có ramp nhẹ bảo vệ bánh răng hộp số JGA25
  if (now - lastRampAt >= RAMP_INTERVAL_MS) {
    lastRampAt = now;
    if (currentSpeed < targetSpeed) currentSpeed = min(currentSpeed + RAMP_STEP, targetSpeed);
    if (currentSpeed > targetSpeed) currentSpeed = max(currentSpeed - RAMP_STEP, targetSpeed);

    // Kiểm tra an toàn trước khi xuất xung PWM
    if ((currentSpeed * MOTOR_DIRECTION > 0 && isLimitFwdHit()) ||
        (currentSpeed * MOTOR_DIRECTION < 0 && isLimitRevHit())) {
      releaseMotor();
      return;
    }

    setMotorPWM(currentSpeed);
  }
}

#ifdef MODE_SELF_TEST
bool checkControls() {
  bool ok = joystickSpeed(0) == -MAX_SPEED && joystickSpeed(1023) == MAX_SPEED;
  ok &= joystickSpeed(431) == 0 && joystickSpeed(511) == 0 && joystickSpeed(591) == 0;
  ok &= joystickSpeed(200) < 0 && joystickSpeed(800) > 0;
  ok &= !buttonPressed(HIGH, 0);
  ok &= !buttonPressed(LOW, 10);
  ok &= !buttonPressed(HIGH, 15);
  ok &= !buttonPressed(LOW, 20);
  ok &= !buttonPressed(LOW, 59);
  ok &= buttonPressed(LOW, 60);
  ok &= !buttonPressed(LOW, 500);
  ok &= !buttonPressed(HIGH, 510);
  ok &= !buttonPressed(HIGH, 550);
  ok &= !buttonPressed(LOW, 600);
  ok &= buttonPressed(LOW, 640);
  lastReading = stableButton = HIGH;
  changedAt = 0;
  return ok;
}
#endif

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LIMIT_FWD_PIN, INPUT_PULLUP);
  pinMode(LIMIT_REV_PIN, INPUT_PULLUP);
  pinMode(MANUAL_LED_PIN, OUTPUT);
  pinMode(AUTO_LED_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  releaseMotor();
  Serial.begin(9600);
#ifdef MODE_SELF_TEST
  Serial.println(checkControls() ? F("CONTROLS TEST PASS") : F("CONTROLS TEST FAIL"));
#endif
  lastReading = stableButton = digitalRead(BUTTON_PIN);
  changedAt = millis();
  showMode();
}

void loop() {
  unsigned long now = millis();
  if (buttonPressed(digitalRead(BUTTON_PIN), now)) {
    autoMode = !autoMode;
    manualReady = false;
    currentSpeed = autoSpeed = 0;
    releaseMotor();
    lastRampAt = now;
    autoStartedAt = now;
    autoDuration = 500;
    if (autoMode) randomSeed(micros());
    showMode();
    return;
  }

  bool hitFwd = isLimitFwdHit();
  bool hitRev = isLimitRevHit();

  int targetSpeed = 0;
  if (autoMode) {
    // Khi chạm công tắc hành trình: tự động dừng chiều đó và lập tức đảo chiều chạy ngược lại
    if (autoSpeed * MOTOR_DIRECTION > 0 && hitFwd) {
      autoSpeed = -MOTOR_DIRECTION * MAX_SPEED; // Đảo chiều lùi ngay
      currentSpeed = 0;
      releaseMotor();
      autoStartedAt = now;
      autoDuration = random(2000, 4001);
    } else if (autoSpeed * MOTOR_DIRECTION < 0 && hitRev) {
      autoSpeed = MOTOR_DIRECTION * MAX_SPEED;  // Đảo chiều tiến ngay
      currentSpeed = 0;
      releaseMotor();
      autoStartedAt = now;
      autoDuration = random(2000, 4001);
    } else if (now - autoStartedAt >= autoDuration) {
      autoStartedAt = now;
      if (autoSpeed != 0) {
        autoSpeed = 0;
        autoDuration = random(400, 801); // Nghỉ ngắn giữa các lần đổi chiều (0.4s - 0.8s)
      } else {
        // Chọn chiều ngẫu nhiên nhưng tránh chiều đang bị kẹt công tắc
        int dir = random(2) ? 1 : -1;
        if (dir * MOTOR_DIRECTION > 0 && hitFwd) dir = -dir;
        if (dir * MOTOR_DIRECTION < 0 && hitRev) dir = -dir;
        autoSpeed = dir * MAX_SPEED; // Chạy hết công suất 100% (255 PWM)
        autoDuration = random(2000, 4001); // Chạy 2s - 4s
      }
    }
    targetSpeed = autoSpeed;
  } else {
    int y = analogRead(A1);
    if (abs(y - CENTER) <= DEAD_ZONE) manualReady = true;
    if (manualReady) targetSpeed = joystickSpeed(y);

    // Safe Stop ở chế độ thủ công:
    // Nếu chạm công tắc đầu nào thì chặn chiều đâm vào đầu đó, nhưng vẫn cho phép gạt lùi ra!
    if (targetSpeed * MOTOR_DIRECTION > 0 && hitFwd) {
      targetSpeed = 0;
      currentSpeed = 0;
    } else if (targetSpeed * MOTOR_DIRECTION < 0 && hitRev) {
      targetSpeed = 0;
      currentSpeed = 0;
    }

    if (targetSpeed == 0) currentSpeed = 0;

    // In thông số debug Joystick qua Serial
    debugJoystick(y, targetSpeed, manualReady, hitFwd, hitRev, now);
  }
  runMotor(targetSpeed, now);
}
