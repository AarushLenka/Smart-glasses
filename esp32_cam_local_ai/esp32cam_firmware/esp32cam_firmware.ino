#include "esp_camera.h"
#include "esp_timer.h"
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "E";
const char* password = "11235813";

// --- Tuning knobs -----------------------------------------------------------
// ESP32 WiFi sustains roughly 2-4 Mbps of HTTP throughput in practice. At 24fps
// that is a ~10-20 KB budget per JPEG. QVGA (320x240) at quality 12 lands there.
// Bumping to FRAMESIZE_HVGA/VGA will silently drop below 24fps on most links --
// watch the "stream: N.N fps" line on serial and back off if it sags.
#define STREAM_FRAMESIZE   FRAMESIZE_QVGA
#define STREAM_JPEG_QUALITY 12          // lower number = better image, bigger frame
#define TARGET_FPS          24
// ---------------------------------------------------------------------------

static const int64_t FRAME_INTERVAL_US = 1000000LL / TARGET_FPS;

WebServer server(80);

// AI Thinker ESP32-CAM Pins
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

void handleRoot() {
  String html =
      "<html><body>"
      "<h2>ESP32 Camera</h2>"
      "<img src='/stream' width='640'>"
      "</body></html>";

  server.send(200, "text/html", html);
}

void handleJPG() {
  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    server.send(500, "text/plain", "Capture failed");
    return;
  }

  WiFiClient client = server.client();
  client.setNoDelay(true);

  char header[160];
  int n = snprintf(header, sizeof(header),
                   "HTTP/1.1 200 OK\r\n"
                   "Content-Type: image/jpeg\r\n"
                   "Content-Length: %u\r\n"
                   "Cache-Control: no-store\r\n"
                   "Connection: close\r\n\r\n",
                   fb->len);

  client.write((const uint8_t *)header, n);
  client.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);
}

void handleStream() {
  WiFiClient client = server.client();
  client.setNoDelay(true);   // no Nagle: a frame must not wait on the next one

  static const char STREAM_HEADER[] =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Cache-Control: no-cache, private\r\n"
      "Pragma: no-cache\r\n"
      "Connection: keep-alive\r\n\r\n";

  if (client.write((const uint8_t *)STREAM_HEADER, sizeof(STREAM_HEADER) - 1) == 0)
    return;

  char part[96];
  int64_t next_frame_us = esp_timer_get_time();
  int64_t report_us = next_frame_us;
  uint32_t frames = 0;

  while (client.connected()) {
    // Pace to a fixed 24fps grid rather than sleeping a constant amount after
    // each send, so capture+encode+transmit jitter does not accumulate.
    int64_t now = esp_timer_get_time();
    int64_t wait = next_frame_us - now;

    if (wait > 1000)
      delay((uint32_t)(wait / 1000));
    else if (wait > 0)
      delayMicroseconds((uint32_t)wait);

    next_frame_us += FRAME_INTERVAL_US;

    // Fell more than one interval behind (WiFi retry burst): resync instead of
    // trying to catch up with a run of back-to-back frames.
    now = esp_timer_get_time();
    if (now - next_frame_us > FRAME_INTERVAL_US)
      next_frame_us = now + FRAME_INTERVAL_US;

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("stream: capture failed");
      continue;
    }

    int n = snprintf(part, sizeof(part),
                     "--frame\r\n"
                     "Content-Type: image/jpeg\r\n"
                     "Content-Length: %u\r\n\r\n",
                     fb->len);

    bool ok = client.write((const uint8_t *)part, n) == (size_t)n &&
              client.write(fb->buf, fb->len) == fb->len &&
              client.write((const uint8_t *)"\r\n", 2) == 2;

    esp_camera_fb_return(fb);

    if (!ok)
      break;

    frames++;
    now = esp_timer_get_time();
    if (now - report_us >= 5000000LL) {
      Serial.printf("stream: %.1f fps\n", frames * 1000000.0 / (now - report_us));
      frames = 0;
      report_us = now;
    }
  }

  client.stop();
}

void setup() {

  Serial.begin(115200);

  camera_config_t config = {};   // must be zeroed: grab_mode/fb_location are read

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = STREAM_FRAMESIZE;
  config.jpeg_quality = STREAM_JPEG_QUALITY;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 2;                      // double buffer: capture overlaps send
    config.grab_mode = CAMERA_GRAB_LATEST;    // always newest frame, never a stale queue
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("Camera Init Failed: 0x%x\n", err);
    while (true);
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    // OV2640 auto-exposure will stretch integration time in dim light and drag
    // the sensor frame rate under 24fps. Cap the AE level to keep it honest.
    s->set_ae_level(s, -1);
    s->set_gain_ctrl(s, 1);
    s->set_whitebal(s, 1);
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);   // modem sleep stalls TCP for a full beacon interval
  WiFi.begin(ssid, password);

  Serial.print("Connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi Connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());

  server.on("/", handleRoot);
  server.on("/jpg", handleJPG);
  server.on("/stream", HTTP_GET, handleStream);

  server.begin();
  Serial.println("Server Started");
}

void loop() {
  server.handleClient();
}
