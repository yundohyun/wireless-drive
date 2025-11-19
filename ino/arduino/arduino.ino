#include <SoftwareSerial.h>

#define ESP_TX 10
#define ESP_RX 11

SoftwareSerial espSerial(ESP_TX, ESP_RX);
int now_input_data = 0;
String input_data = "";

void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);
  Serial.println("Arduino UART Master Ready");
}

void loop() {
  // 시리얼에서 값 입력 시 ESP32-CAM 시리얼로 전달
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0)
      espSerial.println(input);
  }

  // ESP32-CAM 시리얼에서 값을 받을 경우
  while (espSerial.available()) {
    char data = espSerial.read();
    if (now_input_data == 0 && data == '<') {
      input_data = "";
      now_input_data = 1;
    } else if (now_input_data == 1) {
      if (data == '>') now_input_data = 0;
      else input_data += data;
    } else Serial.print(data);
  }

  // Post 데이터 처리
  if (input_data != "" && now_input_data == 0) {
    Serial.print("Recieve Post Data : ");
    Serial.println(input_data);

    // TODO: input_data를 사용하여 제어

    input_data = "";
  }
}
