#include <Arduino.h>
#include "robot_config.h"

namespace {
const MotorPins kMotors[4] = {MOTOR_FRONT_LEFT, MOTOR_REAR_LEFT, MOTOR_FRONT_RIGHT, MOTOR_REAR_RIGHT};
int sensorValues[4] = {0, 0, 0, 0};
bool lineDetected[4] = {false, false, false, false};
float lastError = 0.0f, integral = 0.0f, lastLineSide = 0.0f;
uint32_t lastControlMs = 0, lastLogMs = 0;

int clampPwm(int value) { return constrain(value, -PWM_MAX, PWM_MAX); }
void setMotor(const MotorPins &motor, int speed) {
  speed = clampPwm(speed);
  if (motor.inverted) speed = -speed;
  digitalWrite(motor.in1, speed >= 0 ? HIGH : LOW);
  digitalWrite(motor.in2, speed >= 0 ? LOW : HIGH);
  ledcWrite(motor.pwm, static_cast<uint32_t>(abs(speed)));
}
void setSideSpeeds(int left, int right) {
  setMotor(MOTOR_FRONT_LEFT, left); setMotor(MOTOR_REAR_LEFT, left);
  setMotor(MOTOR_FRONT_RIGHT, right); setMotor(MOTOR_REAR_RIGHT, right);
}
void stopMotors() {
  for (const MotorPins &m : kMotors) {
    digitalWrite(m.in1, LOW); digitalWrite(m.in2, LOW); ledcWrite(m.pwm, 0);
  }
}
void readSensors() {
  for (size_t i = 0; i < 4; ++i) {
    sensorValues[i] = analogRead(SENSOR_PINS[i]);
    const bool high = sensorValues[i] >= SENSOR_THRESHOLD;
    lineDetected[i] = LINE_IS_DARK ? !high : high;
  }
}
bool estimateLinePosition(float &position) {
  constexpr float weights[4] = {-3.0f, -1.0f, 1.0f, 3.0f};
  float sum = 0.0f, count = 0.0f;
  for (size_t i = 0; i < 4; ++i) if (lineDetected[i]) { sum += weights[i]; count += 1.0f; }
  if (count == 0.0f) return false;
  position = sum / count;
  if (position < -0.25f) lastLineSide = -1.0f;
  if (position > 0.25f) lastLineSide = 1.0f;
  return true;
}
void logSensors(bool hasLine, float error) {
  const uint32_t now = millis();
  if (now - lastLogMs < SENSOR_LOG_INTERVAL_MS) return;
  lastLogMs = now;
  Serial.printf("LDR=[%d,%d,%d,%d] detect=[%d,%d,%d,%d] line=%s error=%.2f\n",
    sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3],
    lineDetected[0], lineDetected[1], lineDetected[2], lineDetected[3], hasLine ? "yes" : "lost", error);
}
void controlStep() {
  readSensors(); float error = 0.0f;
  const bool hasLine = estimateLinePosition(error); logSensors(hasLine, error);
  if (!hasLine) {
    integral = 0.0f; const int search = static_cast<int>(SEARCH_SPEED * lastLineSide);
    setSideSpeeds(-search, search); return;
  }
  const float dt = CONTROL_PERIOD_MS / 1000.0f;
  integral = constrain(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  const float derivative = (error - lastError) / dt; lastError = error;
  const float correction = constrain(KP * error + KI * integral + KD * derivative,
                                    -static_cast<float>(MAX_CORRECTION), static_cast<float>(MAX_CORRECTION));
  // A line to the right means turn right: speed up the left wheels.
  setSideSpeeds(clampPwm(BASE_SPEED + static_cast<int>(correction)),
                clampPwm(BASE_SPEED - static_cast<int>(correction)));
}
void initializeMotor(const MotorPins &m) {
  pinMode(m.in1, OUTPUT); pinMode(m.in2, OUTPUT);
  digitalWrite(m.in1, LOW); digitalWrite(m.in2, LOW);
  if (!ledcAttach(m.pwm, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS)) {
    Serial.printf("PWM setup failed on GPIO %u\n", m.pwm);
    while (true) { stopMotors(); delay(1000); }
  }
  ledcWrite(m.pwm, 0);
}
} // namespace

void setup() {
  Serial.begin(115200); analogReadResolution(12);
  for (uint8_t pin : SENSOR_PINS) pinMode(pin, INPUT);
  for (const MotorPins &m : kMotors) initializeMotor(m);
  stopMotors();
  Serial.println("ESP32 4WD line follower ready; lift wheels during initial checks.");
  lastControlMs = millis();
}
void loop() {
  const uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) { lastControlMs = now; controlStep(); }
}
