#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <L3G4200D.h>
#include <string.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// IR
#define RIGHT_IR_LED_PIN 6
#define RIGHT_IR_SENSOR_PIN 7
#define LEFT_IR_LED_PIN 8
#define LEFT_IR_SENSOR_PIN 9

// 모터
#define SERVO_LEFT_PIN 13
#define SERVO_RIGHT_PIN 12

// LED
#define LIGHT_SENSOR_PIN A3

// 지구의 평균 반경 (미터)
#define EARTH_RADIUS_M 6371000.0

Servo servoLeft;
Servo servoRight;

L3G4200D gyroscope;

// GPS
unsigned long gpsLastTime = 0;
float beforeLat = -1, beforeLon = -1;
float nowLat, nowLon;
float targetLat, targetLon;

// 모터 비동기를 위한 변수
unsigned long startTime = 0; // 움직임 시작 시간 (millis() 값)
unsigned long moveDuration = 0; // 움직여야 할 목표 시간 (msTime)
bool isMoving = false; // 현재 움직임이 진행 중인지 여부

// 모터 관련 변수
bool started = false;
const int STOP_PULSE = 1500; // 서보 모터 정지 펄스 폭 (마이크로초)
const int MAX_SPEED = 200;   // 변경됨: 최대 속도 조정 폭 (1500 ± 200)
const int BASE_DRIVE_SPEED = 100; // 변경됨: 직진 시 기본 속도
const float ANGLE_DEADBAND = 5.0; // 이 각도 이하 오차는 직진 유지
const double STOP_DISTANCE_M = 5.0;
const int setpoint = 2;                      // Target distances
const int kpl = -50;                         // Proportional control constants
const int kpr = -50;

// 조도 센서 & LED
const int THRESHOLD_ON = 500;   // 켜지는 밝기
const int THRESHOLD_OFF = 700;  // 꺼지는 밝기
bool isNightMode = false;
const int ledPin = 2;  // 라이트 LED 핀

// // 자이로 & 기울기 센서
// const int tiltPin = 7; // 기울기 센
// float pitch = 0;  
// unsigned long lastTime = 0;
// float downhillSmoothing = 0.9;
// int prevDownhillSpeed = 0;
// float gyroDeadband = 2.0;

// JSON
int json_start = 0, now_input_data = 0;
String input_data = "";

double deg_to_rad(double deg) {
  return deg * (M_PI / 180.0);
}

double rad_to_deg(double rad) {
  return rad * (180.0 / M_PI);
}

double calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
  // 1. 도(Degree)를 라디안(Radian)으로 변환
  double lat1_rad = deg_to_rad(lat1);
  double lon1_rad = deg_to_rad(lon1);
  double lat2_rad = deg_to_rad(lat2);
  double lon2_rad = deg_to_rad(lon2);

  // 2. 경도 차이 계산
  double dLon = lon2_rad - lon1_rad;

  // 3. 방위각 공식의 y와 x 값 계산 (atan2 인자)
  // y = sin(dLon) * cos(lat2)
  double y = sin(dLon) * cos(lat2_rad);
    
  // x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)
  double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dLon);

  // 4. atan2를 사용하여 라디안 방위각 계산
  double bearing_rad = atan2(y, x);

  // 5. 라디안을 도(Degree)로 변환
  double bearing_deg = rad_to_deg(bearing_rad);

  // 6. 결과를 0도에서 360도 사이로 정규화 (방위각)
  // fmod는 실수형의 나머지 연산을 수행합니다.
  return fmod((bearing_deg + 360.0), 360.0);
}

double analyze_movement() {
  double current_movement_direction = calculate_bearing(beforeLat, beforeLon, nowLat, nowLon);
  double direction_to_target = calculate_bearing(nowLat, nowLon, targetLat, targetLon);
  double turn_error = direction_to_target - current_movement_direction;

  // 최단 회전각으로 정규화 (-180° ~ 180°)
  if (turn_error > 180.0) turn_error -= 360.0;
  else if (turn_error < -180.0) turn_error += 360.0;

  return turn_error; // 음수: 왼쪽 회전, 양수: 오른쪽 회전
}

/**
 * @brief 로봇의 회전 각도에 따라 좌우 서보 모터의 속도 조정값(Speed Adjustment)을 계산합니다.
 * @param baseSpeed 직진할 때의 기본 속도 (0 ~ MAX_SPEED 사이의 값)
 * @param turnAngle 목표 회전 각도 (Degree): -90 (최대 왼쪽) ~ 90 (최대 오른쪽)
 * @param leftSpeedRef 왼쪽 모터 속도 조정값 (출력: 0 ~ MAX_SPEED)
 * @param rightSpeedRef 오른쪽 모터 속도 조정값 (출력: 0 ~ MAX_SPEED)
 */
void calculateServoSpeeds(int baseSpeed, float turnAngle, int* leftSpeedRef, int* rightSpeedRef) {
  // 1. 기본 속도 유효성 검사 (0 ~ MAX_SPEED)
  if (baseSpeed < 0) baseSpeed = 0;
  if (baseSpeed > MAX_SPEED) baseSpeed = MAX_SPEED;

  // 2. 각도 유효성 검사 및 정규화 (-90.0 ~ 90.0)
  // 180도 이상 틀어야 할 경우, 90도로 제한하여 회전 우선 모드로 전환 (Spin Turn)
  float limitedAngle = constrain(turnAngle, -90.0, 90.0);

  // 3. 속도 조정값 계산: adjustment는 각도에 비례하여 baseSpeed의 일부를 줄이는 값
  // fabs(limitedAngle / 90.0)는 0.0 ~ 1.0 사이의 값이 됩니다.
  float adjustment = baseSpeed * fabs(limitedAngle / 90.0);

  // 4. 좌우 모터 속도 계산
    
  if (limitedAngle == 0.0 || fabs(limitedAngle) < ANGLE_DEADBAND) {
    // 0도: 직진 또는 오차 범위 내
    *leftSpeedRef = baseSpeed;
    *rightSpeedRef = baseSpeed;
  } else if (limitedAngle > 0.0) {
    // 양수 각도: 오른쪽 회전 (오른쪽 모터 속도를 줄임)
    *leftSpeedRef = baseSpeed;
    *rightSpeedRef = (int)(baseSpeed - adjustment);
  } else { // limitedAngle < 0.0
    // 음수 각도: 왼쪽 회전 (왼쪽 모터 속도를 줄임)
    *leftSpeedRef = (int)(baseSpeed - adjustment);
    *rightSpeedRef = baseSpeed;
  }

  // 결과가 음수가 되지 않도록 보호
  *leftSpeedRef = constrain(*leftSpeedRef, 0, MAX_SPEED);
  *rightSpeedRef = constrain(*rightSpeedRef, 0, MAX_SPEED);
}

// 코드 맨 아래쪽에 추가
double calculate_distance(double lat1, double lon1, double lat2, double lon2) {
  // 1. 도(Degree)를 라디안(Radian)으로 변환
  double lat1_rad = deg_to_rad(lat1);
  double lon1_rad = deg_to_rad(lon1);
  double lat2_rad = deg_to_rad(lat2);
  double lon2_rad = deg_to_rad(lon2);

  // 2. 위도와 경도의 차이
  double dLat = lat2_rad - lat1_rad;
  double dLon = lon2_rad - lon1_rad;

  // 3. 하버사인 공식 (Haversine Formula)의 a값 계산
  // a = sin²(dLat/2) + cos(lat1) * cos(lat2) * sin²(dLon/2)
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1_rad) * cos(lat2_rad) * sin(dLon / 2) * sin(dLon / 2);

  // 4. 중앙각 (Angular Distance) c값 계산
  // c = 2 * atan2(sqrt(a), sqrt(1-a))
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  // 5. 거리 계산 (지구의 평균 반경 EARTH_RADIUS_M = 6371000.0m)
  return EARTH_RADIUS_M * c; // 미터 단위 반환
}

void move(int speedLeft, int speedRight, int msTime) {
  isMoving = true;
  startTime = millis();
  moveDuration = msTime;
  servoLeft.writeMicroseconds(STOP_PULSE + speedLeft);
  servoRight.writeMicroseconds(STOP_PULSE - speedRight);
  delay(msTime);
  Serial.print("move ");
  Serial.print(speedLeft);
  Serial.print(" ");
  Serial.print(speedRight);
  Serial.print(" ");
  Serial.println(msTime);
}

void stop() {
  servoLeft.writeMicroseconds(STOP_PULSE); // 멈춤
  servoRight.writeMicroseconds(STOP_PULSE); // 멈춤
  isMoving = false;
}

int irDistance(int irLedPin, int irReceivePin) {
  int distance = 0;
  for(long f = 38000; f <= 42000; f += 1000)
    distance += irDetect(irLedPin, irReceivePin, f);
  return distance;
}

int irDetect(int irLedPin, int irReceiverPin, long frequency) {
  tone(irLedPin, frequency, 8); // IRLED 38 kHz for at least 1 ms
  delay(1); // Wait 1 ms
  int ir = digitalRead(irReceiverPin); // IR receiver -> ir variable
  delay(1); // Down time before recheck
  return ir; // Return 1 no detect, 0 detect
}

void setup() {
  // 모터 연결
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  // IR
  pinMode(RIGHT_IR_LED_PIN, OUTPUT);
  pinMode(LEFT_IR_LED_PIN, OUTPUT);
  pinMode(RIGHT_IR_SENSOR_PIN, INPUT);
  pinMode(LEFT_IR_SENSOR_PIN, INPUT);
  
  // LED
  pinMode(ledPin, OUTPUT);

  // // 자이로 센서
  // pinMode(tiltPin, INPUT);

  // while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50)) {
  //   Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
  //   delay(500);
  // }
  
  // gyroscope.calibrate();
  // gyroscope.setThreshold(3);

  // lastTime = millis();

  Serial.begin(9600);
}

void loop() {
  // unsigned long now = millis();
  // float dt = (now - lastTime) / 1000.0;
  // lastTime = now;

  while (Serial.available()) {
    char data = Serial.read();

    if (now_input_data == 0 && json_start == 0 && data == '{') {
      json_start = 1;
    } else if (json_start == 1 && data == '"') {
      now_input_data = 1;
      json_start = 0;
    } else if (now_input_data == 1) {
      input_data += data;
      if (data == '}') {
        input_data = "{\"" + input_data;
        now_input_data = 0;
        // Serial.println("JSON Recieved!");
      }
    }
  }
  
  if (millis() - gpsLastTime >= 250) {
    Serial.println("gps");
    delay(10);
    gpsLastTime = millis();
  }

  // Post 데이터 처리
  if (input_data != "" && now_input_data == 0) {
    JsonDocument doc;
    deserializeJson(doc, input_data);
    String type = doc["type"];
    if (type == "start") {
      Serial.println("record_start");
      targetLat = doc["lat"].as<float>();
      targetLon = doc["lon"].as<float>();
      started = true;
      Serial.println("Drive Started!");
      Serial.print("Target - lat: ");
      Serial.print(targetLat);
      Serial.print(", lon: ");
      Serial.println(targetLon);
    } else if (type == "stop" || type == "end") {
      Serial.println("record_stop");
      started = false;
      Serial.println("Drive Stopped!");
    } else if (type == "gps") {
      beforeLat = beforeLat == -1 ? doc["lat"].as<float>() : nowLat;
      beforeLon = beforeLon == -1 ? doc["lon"].as<float>() : nowLon;
      nowLat = doc["lat"].as<float>();
      nowLon = doc["lon"].as<float>();

      Serial.print("Before Location - lat: ");
      Serial.print(beforeLat);
      Serial.print(", lon: ");
      Serial.println(beforeLon);

      Serial.print("Now Location - lat: ");
      Serial.print(nowLat);
      Serial.print(", lon: ");
      Serial.println(nowLon);
    }
    input_data = "";
  }

  // 조도 센서
  int sensorValue = analogRead(LIGHT_SENSOR_PIN);
  if (!isNightMode && sensorValue < THRESHOLD_ON) {
  	isNightMode = true;
  	digitalWrite(ledPin, HIGH);
  } else if (isNightMode && sensorValue > THRESHOLD_OFF) {
    isNightMode = false;
    digitalWrite(ledPin, LOW);
  }

  // // 자이로 센서
  // int tiltState = digitalRead(tiltPin);
  // if (tiltState == LOW) { // 평지일 경우
  //   pitch = 0;
  //   prevDownhillSpeed = 0;
  // }

  // Vector norm = gyroscope.readNormalize();
  // float gyroX = norm.XAxis;
  // if (abs(gyroX) < gyroDeadband) gyroX = 0;
  // pitch += gyroX * dt;
  // int speedupLeft = 0, speedupRight = 0;
  // if (pitch > 10) {
  //   speedupLeft = 50;
  //   speedupRight = 50;
  // } else if (pitch < -10) {
  //   int targetDownhill = -50;

  //   prevDownhillSpeed = (downhillSmoothing * prevDownhillSpeed) +
  //                       ((1.0 - downhillSmoothing) * targetDownhill);

  //   speedupLeft = prevDownhillSpeed;
  //   speedupRight = prevDownhillSpeed;
  // } else {
  //   speedupLeft = 0;
  //   speedupRight = 0;
  //   prevDownhillSpeed = 0;
  // }

  // 모터 센서
  if (isMoving) {
    if (millis() - startTime >= moveDuration) stop();
  } else if (started && beforeLat != -1) {
    double distance_to_target = calculate_distance(nowLat, nowLon, targetLat, targetLon);
    Serial.println(distance_to_target);

    if (distance_to_target < STOP_DISTANCE_M) {
      Serial.println("record_stop");
      Serial.println("Drive Stopped!");
      started = false;
      return; // 이후의 주행 로직을 건너뜁니다.
    }

    double turn_angle = analyze_movement(); 

    int irLeft = irDistance(LEFT_IR_LED_PIN, LEFT_IR_SENSOR_PIN);
    int irRight = irDistance(RIGHT_IR_LED_PIN, RIGHT_IR_SENSOR_PIN);

    if (irLeft != 5 || irRight != 5) {
      int driveLeft = (setpoint - irRight) * kpl;     
      int driveRight = (setpoint - irLeft) * kpr;
      move(100 + driveLeft, 100 + driveRight, 100);
    } else {
      int leftSpeedAdj = 0;
      int rightSpeedAdj = 0;

      calculateServoSpeeds(BASE_DRIVE_SPEED, (float)turn_angle, &leftSpeedAdj, &rightSpeedAdj);
      move(leftSpeedAdj, rightSpeedAdj, 250);

      Serial.print("Turn Angle: ");
      Serial.print(turn_angle, 2);
      Serial.print(" | L Pulse: ");
      Serial.print(STOP_PULSE + leftSpeedAdj);
      Serial.print(", R Pulse: ");
      Serial.print(STOP_PULSE - rightSpeedAdj);
      Serial.println();
    }
  }
}