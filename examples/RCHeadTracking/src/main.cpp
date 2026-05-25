#include <Arduino.h>
#include <M5Unified.h>
#include <SD.h>
#include <Avatar.h>
#include <Stackchan_system_config.h>
#include <Stackchan_servo.h>

using namespace m5avatar;

Avatar avatar;
StackchanSERVO servo;
StackchanSystemConfig system_config;

// PORT B on official StackChan (CoreS3)
const int PIN_CH1 = 8;   // G8  left/right (X axis)
const int PIN_CH2 = 9;   // G9  up/down    (Y axis)

// SCS servo ranges for official StackChan
// Y axis: 5-85 deg recommended (docs warning)
const int SERVO_X_MIN    = 90;
const int SERVO_X_CENTER = 150;
const int SERVO_X_MAX    = 210;
const int SERVO_Y_MIN    = 10;
const int SERVO_Y_CENTER = 45;
const int SERVO_Y_MAX    = 80;

// Futaba standard PWM: 1000-2000 us, center 1500 us
const unsigned long PWM_MIN    = 1000;
const unsigned long PWM_CENTER = 1500;
const unsigned long PWM_MAX    = 2000;
const unsigned long PWM_TIMEOUT = 25000;  // 25 ms timeout (> 1 frame at 50 Hz)

// Deadband around center to suppress stick jitter
const int DEADBAND_X = 8;  // degrees
const int DEADBAND_Y = 4;

// Interrupt-driven PWM measurement (non-blocking)
volatile unsigned long ch1_rise = 0, ch1_width = 0;
volatile unsigned long ch2_rise = 0, ch2_width = 0;

void IRAM_ATTR isr_ch1() {
  if (digitalRead(PIN_CH1)) {
    ch1_rise = micros();
  } else {
    unsigned long w = micros() - ch1_rise;
    if (w > 800 && w < 2200) ch1_width = w;
  }
}

void IRAM_ATTR isr_ch2() {
  if (digitalRead(PIN_CH2)) {
    ch2_rise = micros();
  } else {
    unsigned long w = micros() - ch2_rise;
    if (w > 800 && w < 2200) ch2_width = w;
  }
}

static int applyDeadband(int value, int center, int band) {
  if (abs(value - center) < band) return center;
  return value;
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Log.setLogLevel(m5::log_target_display, ESP_LOG_INFO);
  M5.Log.setEnableColor(m5::log_target_serial, false);

  SD.begin(GPIO_NUM_4, SPI, 25000000);
  delay(2000);
  system_config.loadConfig(SD, "/yaml/SC_BasicConfig.yaml");

  servo.begin(
    system_config.getServoInfo(AXIS_X)->pin,
    system_config.getServoInfo(AXIS_X)->start_degree,
    system_config.getServoInfo(AXIS_X)->offset,
    system_config.getServoInfo(AXIS_Y)->pin,
    system_config.getServoInfo(AXIS_Y)->start_degree,
    system_config.getServoInfo(AXIS_Y)->offset,
    (ServoType)system_config.getServoType());
  delay(2000);
  avatar.init();

  pinMode(PIN_CH1, INPUT);
  pinMode(PIN_CH2, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_CH1), isr_ch1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH2), isr_ch2, CHANGE);

  M5.Display.printf("RC Head Tracking\nCH1->G8  CH2->G9");
  M5_LOGI("RC Head Tracking ready. CH1=G%d CH2=G%d", PIN_CH1, PIN_CH2);
}

void loop() {
  M5.update();

  // Atomically read latest pulse widths captured by ISR
  noInterrupts();
  unsigned long pw_x = ch1_width;
  unsigned long pw_y = ch2_width;
  interrupts();

  if (pw_x > 0 && pw_y > 0) {
    // CH1: stick right -> servo_x increases (look right)
    int servo_x = (int)map((long)pw_x, PWM_MIN, PWM_MAX, SERVO_X_MAX, SERVO_X_MIN);
    // CH2: stick up -> servo_y increases (look up)
    int servo_y = (int)map((long)pw_y, PWM_MIN, PWM_MAX, SERVO_Y_MIN, SERVO_Y_MAX);

    servo_x = constrain(servo_x, SERVO_X_MIN, SERVO_X_MAX);
    servo_y = constrain(servo_y, SERVO_Y_MIN, SERVO_Y_MAX);

    servo_x = applyDeadband(servo_x, SERVO_X_CENTER, DEADBAND_X);
    servo_y = applyDeadband(servo_y, SERVO_Y_CENTER, DEADBAND_Y);

    servo.moveXY(servo_x, servo_y, 50);
  }
}
