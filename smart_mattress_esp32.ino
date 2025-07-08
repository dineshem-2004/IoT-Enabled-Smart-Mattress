#include <WiFi.h>
#include <DHT.h>

#define AD8232_PIN 39
#define LO_MINUS_PIN 16
#define LO_PLUS_PIN 17
#define BREATH_PRESSURE_PIN 32
#define POSITION_PRESSURE_PIN 33
#define TEMP_SENSOR_PIN 14
#define VIBRATION_MOTOR_PIN 27
#define DHTTYPE DHT11

DHT dht(TEMP_SENSOR_PIN, DHTTYPE);

#define BREATH_THRESHOLD 2100
#define POSITION_THRESHOLD 300
#define MONITORING_DURATION 25200 // 7 hours in seconds

unsigned long startTime;
bool goodSleep = false;

// Wi-Fi Credentials
const char* ssid = "Charge_ur_data";
const char* password = "datakallan123";

// Main ESP32 IP and Port
const char* mainESP32IP = "192.168.137.175";
const int mainESP32Port = 12345;

WiFiClient client;
String dataToSend;

// Sensor data variables
unsigned long breathStartTime = 0;
int heartRate = 0;
int breathPressure = 0;
int positionPressure = 0;
float temperature = 0;
int positionChanges = 0;
int breath_alert = 0;

// Sleep position tracking
bool vibratorState = false;
unsigned long positionChangeStartTime = 0;
bool breathAbnormality = false;

void checkBreathAbnormality() {
  breathPressure = analogRead(BREATH_PRESSURE_PIN);

  if (breathPressure > BREATH_THRESHOLD) {
    breath_alert = 0;
    if (!breathAbnormality) {
      breathStartTime = millis();
      breathAbnormality = true;
    }
    if (millis() - breathStartTime >= 60000) {
      Serial.println("Breathing abnormality detected! Pressure > threshold for 1 minute.");
      breath_alert = 1;
      breathAbnormality = false;
    }
  } else {
    breathAbnormality = false;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LO_MINUS_PIN, INPUT);
  pinMode(LO_PLUS_PIN, INPUT);
  pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
  dht.begin();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Connected to Wi-Fi");
  startTime = millis();
}

void loop() {
  unsigned long elapsedTime = (millis() - startTime) / 1000;

  // Read AD8232 heart rate
  bool loPlus = digitalRead(LO_PLUS_PIN);
  bool loMinus = digitalRead(LO_MINUS_PIN);
  if (loPlus || loMinus) {
    Serial.println("Poor Signal Quality from AD8232");
    heartRate = 0;
  } else {
    heartRate = analogRead(AD8232_PIN);
  }

  // Read sensor values
  breathPressure = analogRead(BREATH_PRESSURE_PIN);
  positionPressure = analogRead(POSITION_PRESSURE_PIN);
  temperature = dht.readTemperature();

  // Sleep quality check
  if (elapsedTime < MONITORING_DURATION) {
    if (breathPressure < BREATH_THRESHOLD && positionPressure > POSITION_THRESHOLD) {
      goodSleep = true;
    }
  } else {
    if (goodSleep) {
      Serial.println("Good sleep quality");
    } else {
      Serial.println("Poor sleep quality");
    }
    startTime = millis();
    goodSleep = false;
  }

  // Position change detection
  if (positionPressure > POSITION_THRESHOLD) {
    if (!vibratorState) {
      digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
      vibratorState = true;
      positionChanges++;
      positionChangeStartTime = millis();
    }
  } else {
    if (vibratorState && millis() - positionChangeStartTime > 3000) {
      digitalWrite(VIBRATION_MOTOR_PIN, LOW);
      vibratorState = false;
    }
  }

  checkBreathAbnormality();

  // Prepare data string
  dataToSend = String("HR:") + String(heartRate) +
               ",BP:" + String(breathPressure) +
               ",PP:" + String(positionPressure) +
               ",Temp:" + String(temperature) +
               ",PosChanges:" + String(positionChanges) +
               ",breath_alert:" + String(breath_alert) +
               ",Sleep_qlty:" + String(goodSleep);

  // Send data to main ESP32
  if (!client.connected()) {
    Serial.println("Connecting to main ESP32...");
    if (client.connect(mainESP32IP, mainESP32Port)) {
      Serial.println("Connected to main ESP32");
    } else {
      Serial.println("Connection failed");
      delay(1000);
      return;
    }
  }

  client.println(dataToSend);
  Serial.println("Data sent: " + dataToSend);
  delay(3000); // Send every 3 seconds
}
