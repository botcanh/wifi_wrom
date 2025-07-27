#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Encoder.h>

const char* ssid     = "Tin tin";
const char* password = "629giaiphong";

WiFiServer server(3333);
WiFiClient client;

#define PWM_CHANNEL 0
#define PWM_FREQ 1000
#define PWM_RES 8

const int Pin_IN1 = 18;
const int Pin_IN2 = 19;
const int Pin_PWM = 21;

ESP32Encoder encoder;

float Kp = 2.0, Ki = 1.5, Kd = 0.05;
float setpoint = 150.0, input = 0.0, output = 0.0;
float error = 0.0, last_error = 0.0, integral = 0.0;
unsigned long lastTime = 0;
long lastPosition = 0;
const int stepsPerRevolution = 937;

void setup() {
  pinMode(Pin_IN1, OUTPUT);
  pinMode(Pin_IN2, OUTPUT);

  encoder.attachFullQuad(34, 35);
  encoder.clearCount();

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(Pin_PWM, PWM_CHANNEL);

  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Kết nối Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  lastTime = millis();
}

float computePID(float setpoint, float input, float dt) {
  error = setpoint - input;
  integral += error * dt;
  float derivative = (error - last_error) / dt;
  last_error = error;
  return constrain(Kp * error + Ki * integral + Kd * derivative, -255, 255);
}

void loop() {
  // Kết nối client mới nếu chưa có
  if (!client || !client.connected()) {
    client = server.available();
  }

  // Nhận dữ liệu từ client nếu có
  if (client && client.connected() && client.available()) {
    String msg = client.readStringUntil('\n');
    msg.trim();
    float sp = msg.toFloat();
    if (sp != 0.0 || msg == "0" || msg == "0.0") {
      setpoint = sp;
      Serial.print("Wi-Fi Setpoint: ");
      Serial.println(setpoint);
      client.println("OK: Setpoint = " + String(setpoint));
    } else {
      client.println("Lỗi: Không hợp lệ");
    }
  }

  // Nhận dữ liệu từ Serial nếu cần
  if (Serial.available()) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();
    float sp = inputStr.toFloat();
    if (sp != 0.0 || inputStr == "0" || inputStr == "0.0") {
      setpoint = sp;
    }
  }

  // PID điều khiển mỗi 20ms
  unsigned long now = millis();
  if (now - lastTime >= 20) {
    float dt = (now - lastTime) / 1000.0;

    long currentPosition = encoder.getCount();
    long deltaSteps = currentPosition - lastPosition;
    float revs = deltaSteps / (float)stepsPerRevolution;
    input = revs / (dt / 60.0); // RPM

    output = computePID(setpoint, input, dt);
    int pwmOut = abs(output);

    if (output < 0) {
      digitalWrite(Pin_IN2, LOW);
      digitalWrite(Pin_IN1, HIGH);
    } else if (output > 0) {
      digitalWrite(Pin_IN2, HIGH);
      digitalWrite(Pin_IN1, LOW);
    } else {
      digitalWrite(Pin_IN1, LOW);
      digitalWrite(Pin_IN2, LOW);
    }

    ledcWrite(PWM_CHANNEL, pwmOut);

    Serial.printf("Setpoint: %.2f | RPM: %.2f | Output: %.2f | PWM: %d\n",
                  setpoint, input, output, pwmOut);

    lastTime = now;
    lastPosition = currentPosition;
  }
}
