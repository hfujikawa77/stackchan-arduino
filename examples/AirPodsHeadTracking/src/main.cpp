#include <Arduino.h>
#include <M5Unified.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Avatar.h>
#include <Stackchan_system_config.h>
#include <Stackchan_servo.h>

using namespace m5avatar;

Avatar avatar;
StackchanSERVO servo;
StackchanSystemConfig system_config;
WiFiUDP udp;

const uint16_t UDP_PORT      = 50007;
const int SERVO_X_CENTER     = 90;
const int SERVO_Y_CENTER     = 75;
const int SERVO_X_MIN        = 45;
const int SERVO_X_MAX        = 135;
const int SERVO_Y_MIN        = 50;
const int SERVO_Y_MAX        = 90;
const uint32_t SERVO_MOVE_MS = 80;

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

  wifi_s* wifi_info = system_config.getWiFiSetting();
  WiFi.begin(wifi_info->ssid.c_str(), wifi_info->password.c_str());
  M5.Display.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Display.print(".");
  }
  M5.Display.printf("\nIP: %s\nPort: %d\n",
                    WiFi.localIP().toString().c_str(), UDP_PORT);

  udp.begin(UDP_PORT);
}

void loop() {
  M5.update();

  // Drain all pending packets and act on the latest yaw/pitch only
  bool got_packet     = false;
  float latest_yaw   = 0.0f;
  float latest_pitch = 0.0f;

  int packetSize;
  while ((packetSize = udp.parsePacket()) > 0) {
    char buf[32];
    int len  = udp.read(buf, sizeof(buf) - 1);
    buf[len] = '\0';

    float yaw = 0.0f, pitch = 0.0f;
    if (sscanf(buf, "%f,%f", &yaw, &pitch) == 2) {
      latest_yaw   = yaw;
      latest_pitch = pitch;
      got_packet   = true;
    }
  }

  if (got_packet) {
    // AirPods yaw>0 = left turn  → servo X decreases from center
    // AirPods pitch>0 = look up  → servo Y increases from center
    int servo_x = constrain((int)(SERVO_X_CENTER - latest_yaw),
                            SERVO_X_MIN, SERVO_X_MAX);
    int servo_y = constrain((int)(SERVO_Y_CENTER + latest_pitch * 0.5f),
                            SERVO_Y_MIN, SERVO_Y_MAX);
    servo.moveXY(servo_x, servo_y, SERVO_MOVE_MS);
  }
}
