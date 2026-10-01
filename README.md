# ESP32 4WD Line Following Robot

Arduino firmware for a 4-wheel-drive line follower using an ESP32 DevKit, four analog LDR sensors, and two L298N motor-driver boards. Each H-bridge channel drives one motor. The controller estimates line position from the LDR array and steers with PID control.

## Hardware assumptions

This starter assumes an ESP32 DevKit V1 / ESP32-WROOM-32 (Arduino core 3.x), four analog LDR modules or voltage dividers, two L298N boards, and a dark line on a lighter floor. If your driver differs, verify its pinout and ratings before connecting it. Set sensor threshold and polarity in `include/robot_config.h`.

Four motors can draw high current. Check the motor stall current against the L298N and battery ratings. Do not power motors from the ESP32 or USB. Join ESP32, sensor, and driver grounds. Never let an analog sensor output exceed 3.3 V; ESP32 inputs are not 5 V tolerant.

## Wiring

| Function | ESP32 GPIO | Connect to |
|---|---:|---|
| LDR far left to far right | 34, 35, 36, 39 | Sensor analog outputs, in order |
| Front-left PWM / IN1 / IN2 | 13 / 16 / 17 | Driver 1 ENA / IN1 / IN2 |
| Rear-left PWM / IN3 / IN4 | 14 / 18 / 19 | Driver 1 ENB / IN3 / IN4 |
| Front-right PWM / IN1 / IN2 | 25 / 21 / 22 | Driver 2 ENA / IN1 / IN2 |
| Rear-right PWM / IN3 / IN4 | 26 / 23 / 27 | Driver 2 ENB / IN3 / IN4 |

Remove the ENA/ENB jumpers when using PWM. Use a separate suitable motor battery. For bare LDRs, use one voltage divider per sensor and connect each midpoint to an ADC pin. Divider orientation determines whether the line has a higher or lower reading.

## Build and upload

1. Install PlatformIO (VS Code extension or PlatformIO Core).
2. Open this folder as a PlatformIO project.
3. Connect the ESP32 by USB; set `upload_port` in `platformio.ini` if needed.
4. Upload, then open the serial monitor at 115200 baud.
5. Lift the wheels for initial checks and verify sensor readings and each motor direction.

The firmware uses the ESP32 Arduino core 3.x pin-based `ledcAttach` / `ledcWrite` API.

## Calibrate

Read the four ADC values in the serial monitor over the actual floor and track. Adjust `SENSOR_THRESHOLD` and `LINE_IS_DARK` in `include/robot_config.h`. Start with low `BASE_SPEED`; tune `KP`, `KI`, and `KD` gradually. Reverse individual motor direction with its `MOTOR_*_INVERTED` setting or by swapping motor wires. LDR values vary with ambient light, height, and surface, so calibration is required.

## Behavior and limits

Four threshold readings estimate the line position. PID steering changes left and right wheel speeds, with both motors on each side receiving the same command. If the line is lost, the robot searches toward the last detected side. Sensor values and detections are printed for tuning.

This educational prototype does not include obstacle detection, encoders, battery monitoring, or an emergency stop. Test with wheels raised first; check motor current and driver heating before floor tests. Source and wiring guidance are included; physical hardware and a prebuilt binary are not.

## Files

- `platformio.ini` — PlatformIO ESP32 configuration
- `include/robot_config.h` — pin map, sensor threshold, and PID settings
- `src/main.cpp` — sensor processing and four-motor control

## License

MIT. See `LICENSE`.
