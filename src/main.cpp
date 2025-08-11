#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Encoder.h>
#include <string.h>
//only works with RPM < 60
const char* ssid     = "Tin tin";
const char* password = "629giaiphong";
// for ESP32 pin, do not use 6-11, 34 35 36 39 only for input
WiFiServer server(3333);
WiFiClient client;

// 1st motor
#define PWM_CHANNEL 0
#define PWM_FREQ 1000
#define PWM_RES 10
const int Pin_IN1 = 18;
const int Pin_IN2 = 19;
const int Pin_PWM = 21;
ESP32Encoder encoder; //22,23
float setpoint = 0.0, input = 0.0, output = 0.0;
float error = 0.0, last_error = 0.0, integral = 0.0;
unsigned long lastTime = 0;
long lastPosition = 0;

// 2nd motor
#define PWM_CHANNEL_2 1
#define PWM_FREQ_2 1000
#define PWM_RES_2 10
const int Pin_IN1_2 = 25;
const int Pin_IN2_2  = 26;
const int Pin_PWM_2 = 27;
ESP32Encoder encoder_2; //12,14
float setpoint_2 = 0.0, input_2 = 0.0, output_2 = 0.0;
float error_2 = 0.0, last_error_2 = 0.0, integral_2 = 0.0;
unsigned long lastTime_2 = 0;
long lastPosition_2 = 0;

// 3rd motor
#define PWM_CHANNEL_3 2
#define PWM_FREQ_3 1000
#define PWM_RES_3 10
const int Pin_IN1_3 = 32;
const int Pin_IN2_3  = 33;
const int Pin_PWM_3 = 4;
ESP32Encoder encoder_3;//13,15
float setpoint_3 = 0.0, input_3 = 0.0, output_3 = 0.0;
float error_3 = 0.0, last_error_3 = 0.0, integral_3 = 0.0;
unsigned long lastTime_3 = 0;
long lastPosition_3 = 0;

float Kp = 5.0;
float Ki = 1.2;
float Kd = 0.0;
const int stepsPerRevolution = 937;

void setup() {
  pinMode(Pin_IN1, OUTPUT);
  pinMode(Pin_IN2, OUTPUT);
  pinMode(Pin_IN1_2, OUTPUT);
  pinMode(Pin_IN2_2, OUTPUT);
  pinMode(Pin_IN1_3, OUTPUT);
  pinMode(Pin_IN2_3, OUTPUT);

  encoder.attachFullQuad(22, 23);
  encoder.clearCount();
  encoder_2.attachFullQuad(12, 14);
  encoder_2.clearCount();
  encoder_3.attachFullQuad(13, 15);
  encoder_3.clearCount();

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
float feedforward( float rpm){
  return 0.02096 * rpm * rpm + 0.3603 * rpm + 208.23;
}

float computePID(float setpoint, float input, float dt, int id) {
  if (id == 1){
    error = setpoint - input;
    integral += error * dt;
    float derivative = (error - last_error) / dt;
    last_error = error;
    return constrain(Kp * error + Ki * integral + Kd * derivative, -1024, 1024);
  }else if(id == 2){
    error_2 = setpoint - input;
    integral_2 += error_2 * dt;
    float derivative = (error_2 - last_error_2) / dt;
    last_error_2 = error_2;
    return constrain(Kp * error_2 + Ki * integral_2 + Kd * derivative, -1024, 1024);
  }else if (id == 3){
    error_3 = setpoint - input;
    integral_3 += error_3 * dt;
    float derivative = (error_3 - last_error_3) / dt;
    last_error_3 = error_3;
    return constrain(Kp * error_3 + Ki * integral_2 + Kd * derivative, -1024, 1024);
  }
  return 0.0;
}

void resetPID(bool reset1, bool reset2, bool reset3) {
  if (reset1){
    error = 0.0;
    last_error = 0.0;
    integral = 0.0;
  }
  if (reset2){
    error_2 = 0.0;
    last_error_2 = 0.0;
    integral_2 = 0.0;
  }
  if (reset3){
    error_3 = 0.0;
    last_error_3 = 0.0;
    integral_3 = 0.0;
  }
}

void motor_controller(int id, float input,float setpoint, int dt) {
  output = computePID(setpoint, input, dt, id);
  int pwmOut = abs(output);

  if (id == 1)
  {
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
    pwmOut += feedforward(setpoint);
    ledcWrite(PWM_CHANNEL, pwmOut);
  }else if (id == 2)
  {
    if (output_2 < 0) {
    digitalWrite(Pin_IN2_2, LOW);
    digitalWrite(Pin_IN1_2, HIGH);
    } else if (output > 0) {
      digitalWrite(Pin_IN2_2, HIGH);
      digitalWrite(Pin_IN1_2, LOW);
    } else {
      digitalWrite(Pin_IN1_2, LOW);
      digitalWrite(Pin_IN2_2, LOW);
    }
    pwmOut += feedforward(setpoint_2);
    ledcWrite(PWM_CHANNEL_2, pwmOut);
  }
}

void loop() {
  // Kết nối client mới nếu chưa có
  if (!client || !client.connected()) {
    client = server.available();
  }

  // Nhận dữ liệu từ client nếu có
  if (client && client.connected() && client.available()) {
    char buf[50];
    int len = client.readBytesUntil('\n', buf, sizeof(buf) - 1);
    buf[len] = '\0'; // null terminate

    float sp1, sp2, sp3;
    if (sscanf(buf, "%f,%f,%f", &sp1, &sp2, &sp3) == 3) {
        setpoint   = sp1;
        setpoint_2 = sp2;
        setpoint_3 = sp3;
        resetPID(true, true, true);

        Serial.printf("Wi-Fi Setpoints: M1=%.2f, M2=%.2f, M3=%.2f\n", setpoint, setpoint_2, setpoint_3);
        client.printf("OK: %.2f, %.2f, %.2f\n", setpoint, setpoint_2, setpoint_3);
    } else {
        Serial.println("Error parsing setpoints");
        client.println("Error: Bad format");
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

    output = computePID(setpoint, input, dt, 1);
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
    pwmOut += feedforward(setpoint);
    ledcWrite(PWM_CHANNEL, pwmOut);
    lastTime = now;
    lastPosition = currentPosition;
  }
}
