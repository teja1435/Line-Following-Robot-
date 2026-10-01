#pragma once

#include <Arduino.h>

// ESP32 DevKit / ESP32-WROOM-32; two L298N boards; four analog LDR outputs.
struct MotorPins { uint8_t pwm; uint8_t in1; uint8_t in2; bool inverted; };

constexpr MotorPins MOTOR_FRONT_LEFT  {13, 16, 17, false};
constexpr MotorPins MOTOR_REAR_LEFT   {14, 18, 19, false};
constexpr MotorPins MOTOR_FRONT_RIGHT {25, 21, 22, true};
constexpr MotorPins MOTOR_REAR_RIGHT  {26, 23, 27, true};
constexpr uint8_t SENSOR_PINS[4] = {34, 35, 36, 39}; // left to right

constexpr int SENSOR_THRESHOLD = 1800; // calibrate from serial readings
constexpr bool LINE_IS_DARK = true;
constexpr uint32_t PWM_FREQUENCY_HZ = 18000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr int PWM_MAX = 255;
constexpr int BASE_SPEED = 105;
constexpr int MAX_CORRECTION = 110;
constexpr int SEARCH_SPEED = 80;
constexpr float KP = 42.0f;
constexpr float KI = 0.0f;
constexpr float KD = 18.0f;
constexpr float INTEGRAL_LIMIT = 1.5f;
constexpr uint32_t SENSOR_LOG_INTERVAL_MS = 150;
constexpr uint32_t CONTROL_PERIOD_MS = 10;
