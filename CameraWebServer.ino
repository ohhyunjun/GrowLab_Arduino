#include "esp_camera.h"
#include "WiFi.h"
#include "WebServer.h"
#include "ESPmDNS.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- 🔧 사용자가 수정해야 할 부분 ---
const char* ssid     = "D117-1_2.4G";
const char* password = "d1171d1171";
// ------------------------------------

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

WebServer server(80);

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_VGA;
  config.jpeg_quality = 20;
  config.fb_count     = 1;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode   = CAMERA_GRAB_LATEST;
    Serial.println("[Camera] PSRAM 사용");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[Camera] 초기화 실패: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);

  Serial.println("[Camera] 초기화 완료 (180도 회전)");
  return true;
}

void handleCapture() {
  Serial.println("[Server] GET /capture 수신");

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[Server] 촬영 실패");
    server.send(500, "text/plain", "Capture failed");
    return;
  }

  Serial.printf("[Server] 촬영 완료: %u bytes\n", fb->len);

  server.sendHeader("Content-Type",   "image/jpeg");
  server.sendHeader("Content-Length", String(fb->len));
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);

  esp_camera_fb_return(fb);
}

void handleStatus() {
  String json = "{\"status\":\"ok\",\"free_heap\":";
  json += ESP.getFreeHeap();
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=== ESP32-CAM 서버 모드 시작 ===");

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  setCpuFrequencyMhz(160);

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  if (!initCamera()) {
    Serial.println("[Setup] 카메라 초기화 실패 → 재시작");
    delay(3000);
    ESP.restart();
  }

  WiFi.begin(ssid, password);
  Serial.print("[WiFi] 연결 중");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 30) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[WiFi] 연결 실패 → 재시작");
    ESP.restart();
  }

  Serial.printf("\n[WiFi] 연결 성공 — IP: %s\n", WiFi.localIP().toString().c_str());
  WiFi.setTxPower(WIFI_POWER_11dBm);

  if (MDNS.begin("growlab-cam")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("[mDNS] growlab-cam.local 등록 완료");
  } else {
    Serial.println("[mDNS] 등록 실패 (IP 직접 사용)");
  }

  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/status",  HTTP_GET, handleStatus);
  server.begin();

  Serial.println("[Server] HTTP 서버 시작");
  Serial.println("[Server] GET http://growlab-cam.local/capture");
  Serial.println("=================================\n");
}

void loop() {
  server.handleClient();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] 연결 끊김 → 재연결 시도");
    WiFi.reconnect();
    delay(5000);
  }
}
