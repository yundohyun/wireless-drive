#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>
#include "esp_camera.h"
#include "esp_http_server.h"
HardwareSerial espSerial(2);

// Project
#define PROJECT_NAME                   "WirelessDrive"

// UART
#define ESP_TX 14
#define ESP_RX 15

// Camera Config
#define PWDN_GPIO_NUM                  32
#define RESET_GPIO_NUM                 -1
#define XCLK_GPIO_NUM                  0
#define SIOD_GPIO_NUM                  26
#define SIOC_GPIO_NUM                  27

#define Y9_GPIO_NUM                    35
#define Y8_GPIO_NUM                    34
#define Y7_GPIO_NUM                    39
#define Y6_GPIO_NUM                    36
#define Y5_GPIO_NUM                    21
#define Y4_GPIO_NUM                    19
#define Y3_GPIO_NUM                    18
#define Y2_GPIO_NUM                    5
#define VSYNC_GPIO_NUM                 25
#define HREF_GPIO_NUM                  23
#define PCLK_GPIO_NUM                  22

// 4 for flash led or 33 for normal led
#define LED_GPIO_NUM                   4

// Stream Config
httpd_handle_t server = NULL;
#define PART_BOUNDARY                  "123456789000000000000987654321"
#define _STREAM_CONTENT_TYPE           "multipart/x-mixed-replace;boundary=" PART_BOUNDARY
#define _STREAM_BOUNDARY               "\r\n--" PART_BOUNDARY "\r\n"
#define _STREAM_PART                   "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n"

#define CONFIG_LED_ILLUMINATOR_ENABLED 1

String ssid = "", password = "", ip = "";
int led_duty = 0;
bool isStreaming = false;

// LED Flash Config
#if CONFIG_LED_ILLUMINATOR_ENABLED
#define LED_LEDC_GPIO                  22  //configure LED pin
#define CONFIG_LED_MAX_INTENSITY       255
void enable_led(bool en) {  // Turn LED On or Off
  int duty = en ? led_duty : 0;
  if (en && isStreaming && (led_duty > CONFIG_LED_MAX_INTENSITY)) {
    duty = CONFIG_LED_MAX_INTENSITY;
  }
  ledcWrite(LED_LEDC_GPIO, duty);
  //ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
  //ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
  log_i("Set LED intensity to %d", duty);
}
#endif

// WiFi
bool connect_wifi(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  for (int i = 0; i < 10; i++) {
    delay(500);
    if (WiFi.status() == WL_CONNECTED) break;
  }
  return WiFi.status() == WL_CONNECTED;
}

// Camera
void init_camera() {
  camera_config_t config;
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
  // config.frame_size = FRAMESIZE_VGA;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 16;
  config.fb_count = 2;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (!psramFound()) {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_QVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
      config.fb_count = 1;
    } else {
      config.grab_mode = CAMERA_GRAB_LATEST;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    espSerial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }
  espSerial.println("Camera initialized!");

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_VGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
#if CONFIG_LED_ILLUMINATOR_ENABLED
  ledcAttach(LED_GPIO_NUM, 5000, 8);
#else
  log_i("LED flash is disabled -> CONFIG_LED_ILLUMINATOR_ENABLED = 0");
#endif
#endif
}

// Stream
esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[128];

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

#if CONFIG_LED_ILLUMINATOR_ENABLED
  isStreaming = true;
  enable_led(true);
#endif

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      log_e("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          log_e("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      log_e("Send frame failed");
      break;
    }
    int64_t fr_end = esp_timer_get_time();

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;

    frame_time /= 1000;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
#endif
    log_i(
      "MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)", (uint32_t)(_jpg_buf_len), (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time, avg_frame_time,
      1000.0 / avg_frame_time
    );
  }

#if CONFIG_LED_ILLUMINATOR_ENABLED
  isStreaming = false;
  enable_led(false);
#endif

  return res;
}

esp_err_t post_handler(httpd_req_t *req) {
  char buf[512];
  int total_len = req->content_len;
  int cur_len = 0;
  int ret;
  int start_sent = 0;

  while (cur_len < total_len) {
    int to_read = total_len - cur_len;
    if (to_read > sizeof(buf) - 1) to_read = sizeof(buf) - 1;

    ret = httpd_req_recv(req, buf, to_read);
    if (ret <= 0) {
      if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
      return ESP_FAIL;
    }

    buf[ret] = '\0';
    cur_len += ret;
    
    if (!start_sent) {
      espSerial.print("<");
      start_sent = 1;
    }

    espSerial.print(buf);
  }

  espSerial.println(">"); 

  const char* resp_str = "POST data received";
  httpd_resp_send(req, resp_str, strlen(resp_str));

  return ESP_OK;
}

void init_stream() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 4;

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t stream_uri = {
      .uri       = "/stream",
      .method    = HTTP_GET,
      .handler   = stream_handler,
      .user_ctx  = NULL
    };
    httpd_uri_t post_uri = {
      .uri       = "/post",
      .method    = HTTP_POST,
      .handler   = post_handler,
      .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &stream_uri);
    httpd_register_uri_handler(server, &post_uri);
  }

  espSerial.println("httpd server started.");
}

void setup() {
  espSerial.begin(9600, SERIAL_8N1, ESP_TX, ESP_RX);
  espSerial.println("ESP32-CAM UART Slave Ready");

  while (true) {
    if (espSerial.available()) {
      String input = espSerial.readStringUntil('\n');
      input.trim();

      if (input == "scan") {
        espSerial.println("Found Networks :");
        int n = WiFi.scanNetworks();
        String result = "";
        for (int i = 0; i < n; i++) {
          if (i != 0) espSerial.print("\n");
          espSerial.print(WiFi.SSID(i));
          espSerial.print("(");
          espSerial.print(WiFi.RSSI(i));
          espSerial.print(" dBm)");
        }
        espSerial.println(result);
      }

      if (input.indexOf("ssid=") == 0) {
        ssid = input.substring(5);
        espSerial.printf("WiFi SSID is set to '%s'.\n", ssid.c_str());
      }

      if (input.indexOf("password=") == 0) {
        password = input.substring(9);
        espSerial.printf("WiFi Password is set to '%s'.\n", password.c_str());
      }

      if (input.indexOf("ip=") == 0) {
        int last_segment = ip.toInt();
        if (last_segment >= 0 && last_segment <= 255) {
          ip = input.substring(3);
          espSerial.printf("The last segment of WiFi IP address is set to '%s'.\n", ip.c_str());
        } else
          espSerial.println("Error: Invalid IP segment. Please enter a number between 0 and 255.");
      }

      if (input == "home") {
        ip = 130;
        ssid = "U+Net6238";
        password = "815@J54039";
      }

      if (input == "test") {
        ip = 130;
        ssid = "WirelessDrive";
        password = "WirelessDrive";
      }
    }

    if (!ssid.isEmpty() && !password.isEmpty()) {
      if (!connect_wifi(ssid.c_str(), password.c_str())) {
        ssid = password = ip = "";
        espSerial.println("WiFi Connection Failed.");
      } else {
        espSerial.println("WiFi Connected!");

        if (!ip.isEmpty()) {
          IPAddress currentIP = WiFi.localIP();
          IPAddress gatewayIP = WiFi.gatewayIP();
          IPAddress subnetIP = WiFi.subnetMask();
          IPAddress finalIP(currentIP[0], currentIP[1], currentIP[2], ip.toInt());
          WiFi.disconnect();
          WiFi.config(finalIP, gatewayIP, subnetIP);
          if (!connect_wifi(ssid.c_str(), password.c_str())) {
            ssid = password = ip = "";
            espSerial.println("WiFi Connection With IP Setting Failed.");
            espSerial.print("WiFi IP Address : ");
            espSerial.println(WiFi.localIP());
          } else {
            espSerial.println("WiFi IP Setting Success!");
            break;
          }
        } else break; 
      }
    }
  }
    
  espSerial.print("WiFi IP Address : ");
  espSerial.println(WiFi.localIP());

  init_camera();
  init_stream();
}

void loop() {
  delay(100);
}