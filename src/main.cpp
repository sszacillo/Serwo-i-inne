#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Definicje pinów
#define DOOR_SERVO_PIN 21
#define HORIZONTAL_SERVO_PIN 12
#define VERTICAL_SERVO_PIN 13
#define FAN_1_RELAY_PIN 18
#define FAN_2_RELAY_PIN 19
#define LDR_TR_PIN 34
#define LDR_TL_PIN 35
#define LDR_BR_PIN 32
#define LDR_BL_PIN 33
#define IN1 14
#define IN2 27
#define PIN_BTN 26

// Parametry sieciowe
const char* ssid = "OnePlus 9 Pro";
const char* password = "12345678";
const char* mqtt_server = "156.17.231.93";
const int mqtt_port = 1883;

// Zmienna dla stanu przycisku
bool lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Deklaracja obiektów serwomechanizmów
Servo doorServo, horizontalServo, verticalServo;

// Limity serwomechanizmów
int horizontalServoAngle = 90, verticalServoAngle = 90;
const int servohLimitHigh = 180, servohLimitLow = 65;
const int servovLimitHigh = 120, servovLimitLow = 15;

// Zmienna do przechowywania kąta serwomechanizmu drzwi
int doorServoAngle = 90;

int tr = 0, tl = 0, br = 0, bl = 0;
int tol = 50;
int mode = 0;

// Struktura dla wentylatorów
struct Fan {
  uint8_t pin;
  bool state;
};

Fan fan1 = {FAN_1_RELAY_PIN, false};
Fan fan2 = {FAN_2_RELAY_PIN, false};

WiFiClient espClient;
PubSubClient client(espClient);

void setFanState(Fan &fan, bool state) {
  digitalWrite(fan.pin, state ? LOW : HIGH);
  fan.state = state;
  Serial.println((String)"Fan " + (fan.pin == FAN_1_RELAY_PIN ? "1" : "2") + (state ? " turned ON." : " turned OFF."));
}

void moveMotor(bool clockwise, int duration) {
  digitalWrite(IN1, clockwise ? HIGH : LOW);
  digitalWrite(IN2, !clockwise ? HIGH : LOW);
  vTaskDelay(duration * 1000 / portTICK_PERIOD_MS);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void handleFanControl(const JsonDocument& doc, const char* topic) {
  if (strcmp(topic, "smarthome/control/fan/1/status") == 0) {
    setFanState(fan1, strcmp(doc["status"], "on") == 0);
  } else if (strcmp(topic, "smarthome/control/fan/2/status") == 0) {
    setFanState(fan2, strcmp(doc["status"], "on") == 0);
  }
}

void handleModeControl(const JsonDocument& doc) {
  const char* command = doc["status"];
  if (strcmp(command, "safe") == 0) {
    mode = 1;
    Serial.println("Switched to Safe mode.");
  } else if (strcmp(command, "normal") == 0) {
    mode = 0;
    Serial.println("Switched to Normal (LDR) mode.");
  }
}

void handleGateControl(const JsonDocument& doc) {
  const char* command = doc["command"];
  if (strcmp(command, "open") == 0) {
    moveMotor(true, 3);  // Open the gate
    Serial.println("Gate opened.");
  } else if (strcmp(command, "close") == 0) {
    moveMotor(false, 3);  // Close the gate
    Serial.println("Gate closed.");
  } else if (strcmp(command, "stop") == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    Serial.println("Gate movement stopped.");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.println("Error parsing JSON.");
    return;
  }

  if (strncmp(topic, "smarthome/control/fan/", 21) == 0) {
    handleFanControl(doc, topic);
  } else if (strcmp(topic, "smarthome/control/mode") == 0) {
    handleModeControl(doc);
  } else if (strcmp(topic, "smarthome/control/gate/motor/control") == 0) {
    handleGateControl(doc);
  }
}

void mqttTask(void *pvParameters) {
  while (1) {
    if (!client.connected()) {
      while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32SolarTracker")) {
          Serial.println("connected");
          client.subscribe("smarthome/control/fan/1/status");
          client.subscribe("smarthome/control/fan/2/status");
          client.subscribe("smarthome/control/mode");
          client.subscribe("smarthome/control/gate/motor/control");
        } else {
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" try again in 5 seconds");
          vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
      }
    }
    client.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void servoTask(void *pvParameters) {
  while (1) {
    if (mode == 0) {  // LDR mode
      int avl = (tl + bl) / 2;
      int avr = (tr + br) / 2;
      int dhoriz = avl - avr;

      if (abs(dhoriz) > tol) {
        if (avl > avr) {
          horizontalServoAngle = max(horizontalServoAngle - 1, servohLimitLow);
        } else {
          horizontalServoAngle = min(horizontalServoAngle + 1, servohLimitHigh);
        }
        horizontalServo.write(horizontalServoAngle);
      }

      int avt = (tl + tr) / 2;
      int avd = (bl + br) / 2;
      int dvert = avt - avd;

      if (abs(dvert) > tol) {
        if (avt > avd) {
          verticalServoAngle = min(verticalServoAngle + 1, servovLimitHigh);
        } else {
          verticalServoAngle = max(verticalServoAngle - 1, servovLimitLow);
        }
        verticalServo.write(verticalServoAngle);
      }
    } else {
      horizontalServo.write(90);
      verticalServo.write(90);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void readLDRTask(void *pvParameters) {
  while (1) {
    if (mode == 0) {
      tr = analogRead(LDR_TR_PIN);
      tl = analogRead(LDR_TL_PIN);
      br = analogRead(LDR_BR_PIN);
      bl = analogRead(LDR_BL_PIN);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(9600);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  pinMode(FAN_1_RELAY_PIN, OUTPUT);
  pinMode(FAN_2_RELAY_PIN, OUTPUT);
  digitalWrite(FAN_1_RELAY_PIN, HIGH);
  digitalWrite(FAN_2_RELAY_PIN, HIGH);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  horizontalServo.attach(HORIZONTAL_SERVO_PIN);
  verticalServo.attach(VERTICAL_SERVO_PIN);
  doorServo.attach(DOOR_SERVO_PIN);

  xTaskCreate(mqttTask, "MQTT Task", 4000, NULL, 1, NULL);
  xTaskCreate(servoTask, "Servo Task", 4000, NULL, 1, NULL);
  xTaskCreate(readLDRTask, "Read LDR Task", 2000, NULL, 1, NULL);
}

void loop() {
  // Loop is empty as FreeRTOS handles tasks
}
