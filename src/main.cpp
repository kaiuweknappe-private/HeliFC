#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include "PIDController.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const uint8_t SDA_PIN = 21;
const uint8_t SCL_PIN = 22; 
const uint8_t SERVO1_PIN = 17; 
const uint8_t SERVO2_PIN = 18;
const uint8_t SERVO3_PIN = 19;
const uint8_t MAINESC_PIN = 0;  
const uint8_t TAILESC_PIN = 0;
const uint8_t ULTRASONIC_PIN = 4;
const uint16_t DEFAULT_PULSE_WIDTH = 1500; 

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
imu::Vector<3> euler;

float currentAltitude = 0.0f;
uint32_t readDistance() { 
  pinMode(ULTRASONIC_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_PIN, LOW);

  pinMode(ULTRASONIC_PIN, INPUT);
  uint32_t duration = pulseIn(ULTRASONIC_PIN, HIGH, 30000); // Timeout 30 ms

  uint32_t distance_cm = duration / 58; // Umrechnungsfaktor für cm
  return distance_cm;
}

//PID controllers: 
PIDController RollPid(2.0, 0.0, 0.1);
PIDController PitchPid(2.0, 0.0, 0.1);
PIDController YawPid(1.0, 0.1, 0.01);
PIDController CollectivePid(1.0, 0.0, 0.01);
uint32_t lastMeasurement = micros();
uint32_t lastTrigger = millis();

typedef struct Message {
  char command[5];
} Message;

volatile bool killed = false;
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Message msg;
  memcpy(&msg, incomingData, sizeof(msg));

  // Serial.print("📨 Received: ");
  // Serial.println(msg.command);

  if (strcmp(msg.command, "KILL") == 0) {
    killed = true;
  }
}

void setup() {
  // Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    // Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_register_recv_cb(onReceive);
  // Serial.println(WiFi.macAddress());
  
  // setup imu:
  Wire.begin(SDA_PIN, SCL_PIN);
  // Wire.setClock(400000); 

  delay(1000);
  if (!bno.begin()) {
    while (1) {
      delay(5000);
      // Serial.println("Failed to initialize BNO055! Check wiring or I2C address.");
    }
  }
  bno.setExtCrystalUse(true);
  
  //setup pwm channels:
  uint16_t pwm = map(DEFAULT_PULSE_WIDTH, 0, 20000, 0, 65535); 
  ledcSetup(0, 50, 16); ledcAttachPin(SERVO1_PIN, 0); ledcWrite(0, pwm);
  ledcSetup(1, 50, 16); ledcAttachPin(SERVO2_PIN, 1); ledcWrite(1, pwm);
  ledcSetup(2, 50, 16); ledcAttachPin(SERVO3_PIN, 2); ledcWrite(2, pwm);

  // Serial.println("Setup complete. Starting main loop...");
}

void loop() {
  //todo: funk empfanger/ kill switch
  // read current time
  uint32_t now = micros();
  float dt = (now - lastMeasurement) / 1e6f;
  lastMeasurement = now;
  
  // // read sensor data
  if (now - lastTrigger > 100) { // max 10 Hz
    currentAltitude = readDistance();
    // Serial.println(currentAltitude);
  }

  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float rollCorrection = RollPid.update(0, euler.z(), dt);
  float pitchCorrection = PitchPid.update(0, euler.y(), dt);
  float collectiveCorrection = CollectivePid.update(20, currentAltitude, dt); //target?
  //float yawCorrection = YawPid.update(0, currentYaw, dt);
  // map to [-100, 100] range
  rollCorrection = constrain(rollCorrection * 2, -100, 100);
  pitchCorrection = constrain(pitchCorrection * 2, -100, 100); 
  collectiveCorrection = constrain(collectiveCorrection * 4, -100, 100);

  // swashplate mixing
  float servo1 = collectiveCorrection + pitchCorrection;
  float servo2 = collectiveCorrection - 0.5 * pitchCorrection - 0.866 * rollCorrection;
  float servo3 = collectiveCorrection - 0.5 * pitchCorrection + 0.866 * rollCorrection;
  // todo: Kreiselpraezision beachten

  if(killed) {
    servo1 = 0;
    servo2 = 0;
    servo3 = 0;
  }

  servo1 = map(constrain(1500 + servo1 * 3, 1500, 1800), 0, 20000, 0, 65535);
  servo2 = map(constrain(1500 + servo2 * 3, 1500, 1800), 0, 20000, 0, 65535);
  servo3 = map(constrain(1500 + servo3 * 3, 1500, 1800), 0, 20000, 0, 65535);

  ledcWrite(0, servo1);
  ledcWrite(1, servo2);
  ledcWrite(2, servo3);
  
  delay(40); 
}