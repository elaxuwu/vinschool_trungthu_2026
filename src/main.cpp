#include <Arduino.h>

constexpr uint8_t BUTTON_PIN = 2;
constexpr uint8_t AUTO_LED_PIN = 3;
constexpr uint8_t MANUAL_LED_PIN = 4;
constexpr uint8_t MOTOR_PINS[] = {11, 10, 9, 8}; // IN1, IN2, IN3, IN4
constexpr uint8_t PHASES[] = {0x1, 0x3, 0x2, 0x6, 0x4, 0xC, 0x8, 0x9};
constexpr int CENTER = 511;
constexpr int DEAD_ZONE = 80;
constexpr int MAX_SPEED = 500; // Half-steps/second; tune with the actual motor.
constexpr int MOTOR_DIRECTION = 1; // Set to -1 to reverse physical direction.
constexpr unsigned long DEBOUNCE_MS = 40;

bool autoMode = false;
bool manualReady = false;
int lastReading = HIGH;
int stableButton = HIGH;
unsigned long changedAt = 0;
int currentSpeed = 0;
int autoSpeed = 0;
uint8_t phase = 0;
unsigned long lastStepAt = 0;
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
  if (y < CENTER - DEAD_ZONE)
    return -(long)(CENTER - DEAD_ZONE - y) * MAX_SPEED / (CENTER - DEAD_ZONE);
  if (y > CENTER + DEAD_ZONE)
    return (long)(y - CENTER - DEAD_ZONE) * MAX_SPEED / (1023 - CENTER - DEAD_ZONE);
  return 0;
}

void releaseMotor() {
  for (uint8_t pin : MOTOR_PINS) digitalWrite(pin, LOW);
}

void showMode() {
  digitalWrite(MANUAL_LED_PIN, autoMode ? LOW : HIGH);
  digitalWrite(AUTO_LED_PIN, autoMode ? HIGH : LOW);
  Serial.println(autoMode ? F("MODE=AUTO") : F("MODE=MANUAL"));
}

void runMotor(int targetSpeed, unsigned long now) {
  // Ramp every 10 ms, including slowing through zero before reversing.
  if (now - lastRampAt >= 10) {
    lastRampAt = now;
    if (currentSpeed < targetSpeed) currentSpeed = min(currentSpeed + 5, targetSpeed);
    if (currentSpeed > targetSpeed) currentSpeed = max(currentSpeed - 5, targetSpeed);
  }
  if (currentSpeed == 0) {
    releaseMotor();
    lastStepAt = micros();
    return;
  }
  unsigned long tick = micros();
  unsigned long interval = 1000000UL / abs(currentSpeed);
  if (tick - lastStepAt < interval) return;
  lastStepAt = tick;
  int direction = (currentSpeed > 0 ? 1 : -1) * MOTOR_DIRECTION;
  phase = (phase + (direction > 0 ? 1 : 7)) % 8;
  for (uint8_t i = 0; i < 4; ++i)
    digitalWrite(MOTOR_PINS[i], (PHASES[phase] >> i) & 1);
}

#ifdef MODE_SELF_TEST
// Enable build_flags = -DMODE_SELF_TEST, upload, read the result at 9600 baud.
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
  pinMode(MANUAL_LED_PIN, OUTPUT);
  pinMode(AUTO_LED_PIN, OUTPUT);
  for (uint8_t pin : MOTOR_PINS) pinMode(pin, OUTPUT);
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
    lastStepAt = micros();
    lastRampAt = now;
    autoStartedAt = now;
    autoDuration = 500;
    if (autoMode) randomSeed(micros());
    showMode();
    return;
  }

  int targetSpeed = 0;
  if (autoMode) {
    // Bench demo only: no travel limits until physical end switches are fitted.
    if (now - autoStartedAt >= autoDuration) {
      autoStartedAt = now;
      if (autoSpeed != 0) {
        autoSpeed = 0;
        autoDuration = random(500, 1201);
      } else {
        autoSpeed = random(150, MAX_SPEED + 1) * (random(2) ? 1 : -1);
        autoDuration = random(1000, 3001);
      }
    }
    targetSpeed = autoSpeed;
  } else {
    int y = analogRead(A1);
    if (abs(y - CENTER) <= DEAD_ZONE) manualReady = true;
    if (manualReady) targetSpeed = joystickSpeed(y);
    // Releasing the joystick stops immediately rather than coasting the command.
    if (targetSpeed == 0) currentSpeed = 0;
  }
  runMotor(targetSpeed, now);
}
