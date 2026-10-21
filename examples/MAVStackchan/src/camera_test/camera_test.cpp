// カメラ単体テスト（CoreS3 内蔵 GC0308）。
//   1 枚撮って JPEG に変換し、サイズを表示、画面に表示する。
//   アプリ本体（main.cpp）とは切り離して、環境 camera_test でビルドする。
//   ピン配置は M5Stack 公式の CoreS3 カメラの例に従う。

#include <Arduino.h>
#include <M5Unified.h>
#include "esp_camera.h"
#include "img_converters.h"

namespace {

camera_config_t makeConfig() {
    camera_config_t c = {};
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer = LEDC_TIMER_0;
    c.pin_d0 = 39;
    c.pin_d1 = 40;
    c.pin_d2 = 41;
    c.pin_d3 = 42;
    c.pin_d4 = 15;
    c.pin_d5 = 16;
    c.pin_d6 = 48;
    c.pin_d7 = 47;
    c.pin_xclk = -1;
    c.pin_pclk = 45;
    c.pin_vsync = 46;
    c.pin_href = 38;
    c.pin_sccb_sda = 12;
    c.pin_sccb_scl = 11;
    c.pin_pwdn = -1;
    c.pin_reset = -1;
    c.xclk_freq_hz = 20000000;
    c.pixel_format = PIXFORMAT_RGB565;
    c.frame_size = FRAMESIZE_QVGA;
    c.jpeg_quality = 0;
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    c.sccb_i2c_port = -1;
    return c;
}

}  // namespace

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    Serial.println("[CAM] camera test start");

    const camera_config_t config = makeConfig();
    const esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] init failed: 0x%x\n", err);
        return;
    }
    Serial.println("[CAM] init OK");

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[CAM] capture failed");
        return;
    }
    Serial.printf("[CAM] frame %ux%u, %u bytes\n", fb->width, fb->height, (unsigned)fb->len);

    uint8_t* jpg = nullptr;
    size_t jpg_len = 0;
    const bool ok = frame2jpg(fb, 80, &jpg, &jpg_len);
    esp_camera_fb_return(fb);
    if (!ok || !jpg) {
        Serial.println("[CAM] JPEG conversion failed");
        return;
    }
    Serial.printf("[CAM] JPEG %u bytes\n", (unsigned)jpg_len);

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.drawJpg(jpg, jpg_len, 0, 0);
    free(jpg);
}

void loop() {
    M5.update();
    delay(100);
}
