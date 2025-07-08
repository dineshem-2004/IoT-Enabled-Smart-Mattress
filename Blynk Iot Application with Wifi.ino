#include <WiFi.h>
#include <DHT.h>

// Define sensor and output pins
#define AD8232_PIN 39                 // Heart rate sensor (analog)
#define LO_MINUS_PIN 16              // Lead-off detection pin (-)
#define LO_PLUS_PIN 17               // Lead-off detection pin (+)
#define BREATH_PRESSURE_PIN 32       // Analog pin for breathing pressure sensor
#define POSITION_PRESSURE_PIN 33     // Analog pin for sleep position sensor
#define TEMP_SENSOR_PIN 14           // Temperature sensor (DHT11)
#define VIBRATION_MOTOR_PIN 27       // Output pin for vibration motor

#define DHTTYPE DHT11
DHT dht(TEMP_SENSOR_PIN, DHTTYPE);

// Thresholds for detecting anomalies
#define BREATH_THRESHOLD 2100
#define POSITION_THRESHOLD 300

#define MONITORING_DURATION 25200    // Sleep monitoring time in seconds (7 hours)

unsigned long startTime;
bool goodSleep = false;

// Wi-Fi credentials
const char* ssid = "Charge_ur_data";
const char* password = "datakallan123";

// IP and port of the main ESP32 to send data
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

// For detecting position changes
bool vibratorState = false;
unsigned long positionChangeStartTime = 0;
bool breathAbnormality = false;

// Function to detect abnormal breathing patterns
void checkBreathAbnormality() {
  breathPressure = analogRead(BREATH_PRESSURE_PIN);

  if (breathPressure > BREATH_THRESHOLD) {
    breath_alert = 0;
    if (!breathAbnormality) {
      // Start timing when abnormality begins
      breathStartTime = millis();
      breathAbnormality = true;
    }
    // If pressure remains high for 1 minute, trigger alert
    if (millis() - breathStartTime >= 60000) {
      Serial.println("Breathing abnormality detected! Pressure > threshold for 1 minute.");
      breath_alert = 1;
      breathAbnormality = false;
    }
  } else {
    // Reset if pressure normalizes
    breathAbnormality = false;
  }
}

void setup() {
  Serial.begin(115200);

  // Set input/output modes
  pinMode(LO_MINUS_PIN, INPUT);
  pinMode(LO_PLUS_PIN, INPUT);
  pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
  
  dht.begin();  // Start temperature sensor

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Connected to Wi-Fi");
  
  startTime = millis();  // Start timer
}

void loop() {
  unsigned long elapsedTime = (millis() - startTime) / 1000;  // Time since monitoring started (in seconds)

  // Read heart rate sensor
  bool loPlus = digitalRead(LO_PLUS_PIN);
  bool loMinus = digitalRead(LO_MINUS_PIN);
  if (loPlus || loMinus) {
    Serial.println("Poor Signal Quality from AD8232");
    heartRate = 0;
  } else {
    heartRate = analogRead(AD8232_PIN);
  }

  // Read sensor data
  breathPressure = analogRead(BREATH_PRESSURE_PIN);
  positionPressure = analogRead(POSITION_PRESSURE_PIN);
  temperature = dht.readTemperature();

  // Monitor sleep quality during set duration
  if (elapsedTime < MONITORING_DURATION) {
    if (breathPressure < BREATH_THRESHOLD && positionPressure > POSITION_THRESHOLD) {
      goodSleep = true;
    }
  } else {
    // Sleep result after 7 hours
    if (goodSleep) {
      Serial.println("Good sleep quality");
    } else {
      Serial.println("Poor sleep quality");
    }
    // Reset monitoring
    startTime = millis();
    goodSleep = false;
  }

  // Check for body movement (position pressure)
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

  // Monitor breathing abnormality
  checkBreathAbnormality();

  // Format data string to send to main ESP32
  dataToSend = String("HR:") + String(heartRate) +
               ",BP:" + String(breathPressure) +
               ",PP:" + String(positionPressure) +
               ",Temp:" + String(temperature) +
               ",PosChanges:" + String(positionChanges) +
               ",breath_alert:" + String(breath_alert) +
               ",Sleep_qlty:" + String(goodSleep);

  // Connect and send data via TCP to main ESP32
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

  // Send the data
  client.println(dataToSend);
  Serial.println("Data sent: " + dataToSend);

  delay(3000);  // Wait 3 seconds before next send
}
