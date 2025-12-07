#include <WiFi.h>
#include <string.h>
#include <TinyGPS.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "SD_MMC.h"
HardwareSerial gpsSerial(2);

// Project
#define PROJECT_NAME                   "WirelessDrive"

// UART
#define GPS_TX 0

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

#define CONFIG_LED_ILLUMINATOR_ENABLED 0

bool isRecording = false, gps_connected = false;
unsigned long lastFrameTime = 0;
const unsigned long frameInterval = 100;
float lat, lon;
int frameIndex = 0, led_duty = 0;
String uuid = "";
TinyGPS gps;
float flat, flon;
unsigned long chars = 0;
httpd_handle_t server = NULL;

// LED Flash Config
#if CONFIG_LED_ILLUMINATOR_ENABLED
#define LED_LEDC_GPIO                  22  //configure LED pin
#define CONFIG_LED_MAX_INTENSITY       255
void enable_led(bool en) {  // Turn LED On or Off
  int duty = en ? led_duty : 0;
  if (en && (led_duty > CONFIG_LED_MAX_INTENSITY)) {
    duty = CONFIG_LED_MAX_INTENSITY;
  }
  ledcWrite(LED_LEDC_GPIO, duty);
  //ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
  //ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
  log_i("Set LED intensity to %d", duty);
}
#endif

// WiFi
bool connect_wifi(const char* ssid, const char* password, int ip) {
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  for (int i = 0; i < 10; i++) {
    delay(500);
    if (WiFi.status() == WL_CONNECTED) break;
  }
  if (ip != -1) {
    IPAddress currentIP = WiFi.localIP();
    IPAddress gatewayIP = WiFi.gatewayIP();
    IPAddress subnetIP = WiFi.subnetMask();
    IPAddress finalIP(currentIP[0], currentIP[1], currentIP[2], ip);
    WiFi.disconnect();
    WiFi.config(finalIP, gatewayIP, subnetIP);
    return connect_wifi(ssid, password, -1);
  } else return WiFi.status() == WL_CONNECTED;
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
  // config.frame_size = FRAMESIZE_QVGA;
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
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
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }
  Serial.println("Camera initialized!");

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

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
#if CONFIG_LED_ILLUMINATOR_ENABLED
  ledcAttach(LED_GPIO_NUM, 5000, 8);
#else
  log_i("LED flash is disabled -> CONFIG_LED_ILLUMINATOR_ENABLED = 0");
#endif
#endif
}

String generate_uuid() {
  const char *hex = "0123456789abcdef";
  uint8_t uuid[16];

  // ESP32는 esp_random() 사용
  for (int i = 0; i < 16; i++) {
    uuid[i] = esp_random() & 0xFF;
  }

  // UUID v4 규격 적용
  uuid[6] = (uuid[6] & 0x0F) | 0x40;  // Version 4
  uuid[8] = (uuid[8] & 0x3F) | 0x80;  // Variant

  String out = "";
  for (int i = 0; i < 16; i++) {
    out += hex[(uuid[i] >> 4) & 0x0F];
    out += hex[uuid[i] & 0x0F];

    if (i == 3 || i == 5 || i == 7 || i == 9) out += "-";
  }
  return out;
}

void init_sdcard() {
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card found");
    return;
  }
  Serial.println("SD Card initialized.");
  SD_MMC.mkdir("/wireless-drive");
}

void start_recording() {
  if (isRecording) return;
  frameIndex = 0;
  uuid = generate_uuid();
  isRecording = true;

  char foldername[128];
  sprintf(foldername, "/wireless-drive/%s", uuid.c_str());
  SD_MMC.mkdir(foldername);

  char gpsname[256];
  sprintf(gpsname, "/wireless-drive/%s/gps.json", uuid.c_str());
  File file = SD_MMC.open(gpsname, FILE_WRITE);
  file.print("[");
  file.close();

  Serial.println("Recording Started.");
}

void stop_recording() {
  if (!isRecording) return;
  isRecording = false;

  char filename[256];
  sprintf(filename, "/wireless-drive/%s/gps.json", uuid.c_str());
  File file = SD_MMC.open(filename, FILE_APPEND);
  file.print("]");
  file.close();

  Serial.println("Recording Stopped.");
}

void save_frame_to_sd() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Frame capture failed");
    return;
  }

  char filename[256];
  sprintf(filename, "/wireless-drive/%s/frame_%08d.jpg", uuid.c_str(), frameIndex++);

  File file = SD_MMC.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("File open failed");
  } else {
    file.write(fb->buf, fb->len);
    file.close();
    Serial.printf("Saved: %s\n", filename);
  }

  esp_camera_fb_return(fb);
}

unsigned long get_gps_timestamp(TinyGPS &gps) {
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

  if (age == TinyGPS::GPS_INVALID_AGE) return 0;

  // 월 보정용 배열
  static const int daysBeforeMonth[] = {0,31,59,90,120,151,181,212,243,273,304,334};

  // 윤년 체크
  bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  int days = daysBeforeMonth[month - 1] + (leap && month > 2 ? 1 : 0);

  // 1970년 이후 총 일수 계산
  unsigned long daysSince1970 = 
    (year - 1970) * 365 + 
    (year - 1969) / 4 - 
    (year - 1901) / 100 + 
    (year - 1601) / 400 + 
    days + 
    (day - 1);

  unsigned long seconds =
    daysSince1970 * 86400UL +
    hour * 3600UL +
    minute * 60UL +
    second;

  // 밀리초 추가
  return seconds * 1000UL + (hundredths * 10);
}

void save_gps_info() {
  char filename[256];
  sprintf(filename, "/wireless-drive/%s/gps.json", uuid.c_str());

  File file = SD_MMC.open(filename, FILE_APPEND);
  if (!file) {
    Serial.println("File open failed");
  } else {
    file.printf("%s{\"timestamp\":%lu,\"lat\":%.6f,\"lon\":%.6f}", frameIndex == 0 ? "" : ",", get_gps_timestamp(gps), lat, lon);
    file.close();
  }
}

bool remove_dir_recursive(fs::FS &fs, const char *path) {
  File dir = fs.open(path);
  if (!dir) {
    Serial.println("Failed to open dir");
    return false;
  }
  if (!dir.isDirectory()) {
    Serial.println("Not a directory");
    dir.close();
    return false;
  }
  File file = dir.openNextFile();
  while (file) {
    String filePath = String(path) + "/" + file.name();
    if (file.isDirectory()) {
      remove_dir_recursive(fs, filePath.c_str());
      fs.rmdir(filePath.c_str());
    } else fs.remove(filePath.c_str());

    file = dir.openNextFile();
  }
  dir.close();
  return fs.rmdir(path);
}

static const char *TAG = "HTTP_HANDLER";

// 전역으로 설정할 수도 있지만, 필요한 함수 내에서 정의하는 것이 더 안전함
// File file = File(); // SD_MMC.h/File 객체를 사용한다고 가정

/**
 * @brief HTTP 요청의 URL 쿼리에서 지정된 파일을 찾아 청크 방식으로 응답합니다.
 * * @param req HTTP 요청 구조체
 * @param key_name 파일 경로를 구성하는 데 사용될 쿼리 키 이름 (예: "id")
 * @param file_path_format 파일 경로 포맷 문자열 (예: "/wireless-drive/%s/gps.json")
 * @param content_type 응답 Content-Type (예: "application/json" 또는 "image/jpeg")
 * @param second_key_name 선택적 두 번째 쿼리 키 이름 (예: "frame")
 * @return esp_err_t ESP_OK 또는 오류 코드
 */
esp_err_t file_delivery_handler(
  httpd_req_t *req,
  const char *key_name,
  const char *file_path_format,
  const char *content_type,
  const char *second_key_name
) {
  esp_err_t ret = ESP_FAIL;
  char *query_buf = NULL;
  File file = File();
  char *chunk_buf = NULL;
  const size_t CHUNK_SIZE = 1024; // 청크 버퍼 크기

  do {
    // 1. URL 쿼리 길이 확인
    size_t buf_len = httpd_req_get_url_query_len(req);
    if (buf_len == 0) {
      httpd_resp_send_404(req);
      ret = ESP_OK; // 요청이 유효하지 않아 404를 보냄
      break;
    }

    // 2. 쿼리 버퍼 할당
    query_buf = (char*)malloc(buf_len + 1);
    if (query_buf == NULL) {
      ESP_LOGE(TAG, "OOM: Failed to allocate query buffer.");
      httpd_resp_send_500(req);
      break;
    }

    // 3. 쿼리 문자열 가져오기
    if (httpd_req_get_url_query_str(req, query_buf, buf_len + 1) != ESP_OK) {
      httpd_resp_send_500(req);
      break;
    }

    // 4. 필수 쿼리 키/값 추출
    char primary_value[100]; // key_name에 해당하는 값 (예: ID)
    char secondary_value[100] = {0}; // second_key_name에 해당하는 값 (예: frame)

    if (httpd_query_key_value(query_buf, key_name, primary_value, sizeof(primary_value)) != ESP_OK) {
      httpd_resp_send_404(req);
      ret = ESP_OK;
      break;
    }

    bool secondary_key_required = (second_key_name != NULL);
    if (secondary_key_required) {
      if (httpd_query_key_value(query_buf, second_key_name, secondary_value, sizeof(secondary_value)) != ESP_OK) {
        httpd_resp_send_404(req);
        ret = ESP_OK;
        break;
      }
    }

    // 5. 파일 경로 생성 및 파일 열기
    char filename[256];
    int path_len;
        
    if (secondary_key_required) {
      // "frame"이 필요한 경우 (image_handler 로직)
      path_len = snprintf(filename, sizeof(filename), file_path_format, primary_value, secondary_value);
    } else {
      // "frame"이 필요 없는 경우 (gps_handler 로직)
      path_len = snprintf(filename, sizeof(filename), file_path_format, primary_value);
    }

    // 경로 길이 확인
    if (path_len >= sizeof(filename) || path_len < 0) {
      ESP_LOGE(TAG, "File path too long or error.");
      httpd_resp_send_500(req);
      break;
    }
        
    file = SD_MMC.open(filename);
    if (!file) {
      ESP_LOGW(TAG, "File not found: %s", filename);
      httpd_resp_send_404(req);
      ret = ESP_OK;
      break;
    }

    // 6. 응답 헤더 설정
    httpd_resp_set_type(req, content_type);
        
    // Content-Length 설정 (이미지 전송 시 유용)
    if (file.size() > 0) {
      char content_len_str[32];
      snprintf(content_len_str, sizeof(content_len_str), "%zu", file.size());
      httpd_resp_set_hdr(req, "Content-Length", content_len_str);
    }

    // 7. 청크 버퍼 할당
    chunk_buf = (char*)malloc(CHUNK_SIZE);
    if (chunk_buf == NULL) {
      ESP_LOGE(TAG, "OOM: Failed to allocate chunk buffer.");
      httpd_resp_send_500(req);
      break;
    }

    // 8. 파일 전송 (청크 방식)
    size_t read_bytes;
    ret = ESP_OK; // 전송 시작은 성공으로 가정

    while (file.available()) {
      read_bytes = file.read((uint8_t*)chunk_buf, CHUNK_SIZE);
      if (httpd_resp_send_chunk(req, chunk_buf, read_bytes) != ESP_OK) {
        ESP_LOGE(TAG, "Chunk send failed, aborting transmission.");
        ret = ESP_FAIL; // 전송 실패
        break;
      }
    }

    // 전송 완료를 위해 빈 청크 전송 (전송 실패가 없었을 경우)
    if (ret == ESP_OK) ret = httpd_resp_send_chunk(req, NULL, 0);
  } while (0);

  // 9. 자원 해제 (Cleanup)
  if (chunk_buf) free(chunk_buf);
  if (file) file.close();
  if (query_buf) free(query_buf);
    
  return ret;
}

esp_err_t info_handler(httpd_req_t *req) {
  const char* json_type = "application/json";
  httpd_resp_set_type(req, json_type);

  JsonDocument doc;
  doc["gps"] = gps_connected;
  doc["lat"] = lat;
  doc["lon"] = lon;
  doc["recording"] = isRecording;
  doc["last_id"] = uuid.c_str();
  doc["frame"] = frameIndex;

  char output[256];
  serializeJson(doc, output);

  httpd_resp_send(req, output, strlen(output));

  return ESP_OK;
}

esp_err_t gps_handler(httpd_req_t *req) {
  const char *key = "id";
  const char *format = "/wireless-drive/%s/gps.json";
  const char *mime = "application/json";
    
  return file_delivery_handler(req, key, format, mime, NULL);
}

esp_err_t image_handler(httpd_req_t *req) {
  const char *id_key = "id";
  const char *frame_key = "frame";
  const char *format = "/wireless-drive/%s/frame_%08s.jpg"; 
  const char *mime = "image/jpeg";
    
  return file_delivery_handler(req, id_key, format, mime, frame_key);
}

esp_err_t post_handler(httpd_req_t *req) {
  int total_len = req->content_len;
  int cur_len = 0;
  int ret;

  // JSON 전체를 저장할 버퍼 (최대 4KB 예시)
  static char json_body[4096];
  int json_pos = 0;

  char buf[512];

  while (cur_len < total_len) {
    int to_read = total_len - cur_len;
    if (to_read > sizeof(buf) - 1)
      to_read = sizeof(buf) - 1;

    ret = httpd_req_recv(req, buf, to_read);
    if (ret <= 0) {
      if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
      return ESP_FAIL;
    }

    buf[ret] = '\0';
    cur_len += ret;

    // JSON 전체를 하나의 문자열로 저장
    if (json_pos + ret < sizeof(json_body) - 1) {
      memcpy(json_body + json_pos, buf, ret);
      json_pos += ret;
    }
  }

  // Null-terminate
  json_body[json_pos] = '\0';

  // 시리얼에 JSON 출력
  Serial.println("Received JSON:");
  Serial.println(json_body);

  // 임의의 HTTP 응답
  const char* json_type = "application/json";
  httpd_resp_set_type(req, json_type);
  String response = "{\"status\": \"ok\"}";
  httpd_resp_send(req, response.c_str(), response.length());

  return ESP_OK;
}

esp_err_t record_handler(httpd_req_t *req) {
  if (!isRecording) start_recording();
  else stop_recording();

  const char* json_type = "application/json";
  httpd_resp_set_type(req, json_type);

  JsonDocument doc;
  doc["status"] = "ok";
  doc["recording"] = isRecording;
  doc["id"] = uuid;

  char output[256];
  serializeJson(doc, output);

  httpd_resp_send(req, output, strlen(output));

  return ESP_OK;
}

esp_err_t list_handler(httpd_req_t *req) {
  const char* json_type = "application/json";
  httpd_resp_set_type(req, json_type);

  String result = "[";
  int count = 0;

  File root = SD_MMC.open("/wireless-drive");
  if (root && root.isDirectory()) {
    File file = root.openNextFile();
    while (file) {
      if (file.isDirectory()) {
        if (count++ != 0) result += ",";
        result += "\"";
        result += file.name();
        result += "\"";
      }
      file = root.openNextFile();
    }
    result += "]";
    file.close();
  }
  root.close();

  httpd_resp_send(req, result.c_str(), strlen(result.c_str()));

  return ESP_OK;
}

esp_err_t remove_handler(httpd_req_t *req) {
  const char* json_type = "application/json";
  httpd_resp_set_type(req, json_type);

  JsonDocument doc;
  doc["status"] = remove_dir_recursive(SD_MMC, "/wireless-drive");
  SD_MMC.mkdir("/wireless-drive");

  char output[25];
  serializeJson(doc, output);
  
  httpd_resp_send(req, output, strlen(output));

  return ESP_OK;
}

void init_stream() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 7;

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t info_uri = {
      .uri       = "/",
      .method    = HTTP_GET,
      .handler   = info_handler,
      .user_ctx  = NULL
    };
    httpd_uri_t list_uri = {
      .uri       = "/list",
      .method    = HTTP_GET,
      .handler   = list_handler,
      .user_ctx  = NULL
    };
    httpd_uri_t remove_uri = {
      .uri       = "/remove",
      .method    = HTTP_GET,
      .handler   = remove_handler,
      .user_ctx  = NULL
    };
    httpd_uri_t gps_uri = {
      .uri       = "/gps",
      .method    = HTTP_GET,
      .handler   = gps_handler,
      .user_ctx  = NULL
    };
    httpd_uri_t image_uri = {
      .uri       = "/image",
      .method    = HTTP_GET,
      .handler   = image_handler,
      .user_ctx  = NULL
    };
    httpd_uri_t record_uri = {
      .uri       = "/record",
      .method    = HTTP_GET,
      .handler   = record_handler,
      .user_ctx  = NULL
    };
    httpd_uri_t post_uri = {
      .uri       = "/",
      .method    = HTTP_POST,
      .handler   = post_handler,
      .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &info_uri);
    httpd_register_uri_handler(server, &list_uri);
    httpd_register_uri_handler(server, &remove_uri);
    httpd_register_uri_handler(server, &gps_uri);
    httpd_register_uri_handler(server, &image_uri);
    httpd_register_uri_handler(server, &record_uri);
    httpd_register_uri_handler(server, &post_uri);
  }

  Serial.println("httpd server started.");
}

void setup() {
  gpsSerial.begin(9600, SERIAL_8N1, GPS_TX, -1);
  Serial.begin(9600);
  Serial.println("ESP32 Ready");

  connect_wifi("WirelessDrive", "WirelessDrive", 200);
    
  Serial.print("WiFi IP Address : ");
  Serial.println(WiFi.localIP());

  init_camera();
  init_sdcard();
  init_stream();
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    gps.f_get_position(&lat, &lon, NULL);
    if (!gps_connected) {
      gps.stats(&chars, NULL, NULL);
      if (chars > 0 && lat != TinyGPS::GPS_INVALID_F_ANGLE && lon != TinyGPS::GPS_INVALID_F_ANGLE) {
        gps_connected = true;
        Serial.println("GPS Connected!");
      }
    }
  }

  while (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "record_start") start_recording();
    if (input == "record_stop") stop_recording();
    if (input == "gps") {
      JsonDocument doc;
      doc["type"] = "gps";
      doc["connected"] = gps_connected;
      doc["lat"] = lat;
      doc["lon"] = lon;
    
      char output[256];
      serializeJson(doc, output);

      Serial.println(output);
    }
  }

  if (isRecording) {
    unsigned long now = millis();
    if (now - lastFrameTime >= frameInterval) {
      lastFrameTime = now;
      save_gps_info();
      save_frame_to_sd();
    }
  }
}