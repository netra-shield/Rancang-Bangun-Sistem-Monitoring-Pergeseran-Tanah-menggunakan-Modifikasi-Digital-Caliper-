#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

// ==================== MQTT CONFIGURATION ====================
const char* ssid = "Kampret";
const char* password = "Akha11213";
const char* mqtt_server = "mqtt-dashboard.com";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ==================== MQTT TOPICS ====================
const char* TOPIC_PASAK_DATA = "pasak/sensor_data";

// ==================== PIN I2C untuk ESP32-C3 ====================
#define I2C_SDA 8
#define I2C_SCL 9

// ==================== MPU6050 OBJECT ====================
MPU6050 mpu6050(Wire);

// ==================== FILTER STANDAR CLASS ====================
class SimpleFilter {
public:
  SimpleFilter(float alpha = 0.1) {
    this->alpha = alpha;
    filteredValue = 0.0;
    initialized = false;
  }
  
  float update(float newValue) {
    if (!initialized) {
      filteredValue = newValue;
      initialized = true;
    } else {
      // Simple Low-pass Filter: y(n) = α * x(n) + (1-α) * y(n-1)
      filteredValue = alpha * newValue + (1.0 - alpha) * filteredValue;
    }
    return filteredValue;
  }
  
  void reset(float newValue) {
    filteredValue = newValue;
    initialized = true;
  }
  
  void setAlpha(float newAlpha) {
    alpha = newAlpha;
  }
  
private:
  float alpha;
  float filteredValue;
  bool initialized;
};

// ==================== GLOBAL VARIABLES ====================
SimpleFilter filterPitch(0.1);   // Alpha = 0.1 untuk smoothing medium
SimpleFilter filterRoll(0.1);
SimpleFilter filterYaw(0.1);

float pitchOffset = 0, rollOffset = 0, yawOffset = 0;
unsigned long lastTime = 0;
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 200; // Kirim setiap 200ms

// Data sensor
float filteredPitch = 0, filteredRoll = 0, filteredYaw = 0;
float rawPitch = 0, rawRoll = 0, rawYaw = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

uint32_t packetCounter = 0;
bool mqttConnected = false;
bool mpuInitialized = false;

// Buffer untuk JSON
char jsonBuffer[512];

// ==================== MQTT FUNCTIONS ====================
void setupWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < 15000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ WiFi connection failed");
  }
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    String clientId = "Pasak-";
    clientId += String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("✅ MQTT connected");
      mqttConnected = true;
    } else {
      Serial.print("❌ MQTT connection failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      mqttConnected = false;
      delay(5000);
    }
  }
}

void publishSensorData() {
  if (!mqttConnected) return;
  
  // Format data sebagai JSON manual tanpa ArduinoJson
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f,"
    "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
    "\"packet\":%lu,\"timestamp\":%lu}",
    filteredPitch, filteredRoll, filteredYaw,
    accelX, accelY, accelZ,
    gyroX, gyroY, gyroZ,
    packetCounter, millis()
  );
  
  if (mqttClient.publish(TOPIC_PASAK_DATA, jsonBuffer)) {
    packetCounter++;
    
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 2000) {
      Serial.println("📤 Published to MQTT");
      Serial.print("  Pitch: "); Serial.print(filteredPitch, 2);
      Serial.print("°, Roll: "); Serial.print(filteredRoll, 2);
      Serial.print("°, Yaw: "); Serial.print(filteredYaw, 2); Serial.println("°");
      lastDebug = millis();
    }
  } else {
    Serial.println("❌ Failed to publish MQTT data");
  }
}

// ==================== MPU6050 INITIALIZATION & CALIBRATION ====================
bool initializeMPU6050() {
  Serial.println("🔧 Initializing MPU6050...");
  
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Cek koneksi MPU6050
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() != 0) {
    Serial.println("❌ MPU6050 not found! Check I2C connections.");
    return false;
  }
  
  mpu6050.begin();
  
  // Kalibrasi gyro
  Serial.println("📏 Calibrating gyro... DON'T MOVE!");
  delay(3000);
  mpu6050.calcGyroOffsets(true);
  
  // Leveling - baca posisi awal sebagai referensi 0°
  Serial.println("🎯 Leveling...");
  delay(1000);
  
  // Average beberapa sample untuk akurasi
  float pitchSum = 0, rollSum = 0, yawSum = 0;
  int samples = 100;
  
  for(int i = 0; i < samples; i++) {
    mpu6050.update();
    pitchSum += mpu6050.getAngleX();
    rollSum += mpu6050.getAngleY();
    yawSum += mpu6050.getAngleZ();
    delay(10);
  }
  
  pitchOffset = pitchSum / samples;
  rollOffset = rollSum / samples;
  yawOffset = yawSum / samples;
  
  // Reset filter dengan nilai awal
  filterPitch.reset(0);
  filterRoll.reset(0);
  filterYaw.reset(0);
  
  Serial.println("✅ Calibration and Leveling completed!");
  Serial.print("📐 Offset - Pitch: "); Serial.print(pitchOffset, 2);
  Serial.print("°, Roll: "); Serial.print(rollOffset, 2);
  Serial.print("°, Yaw: "); Serial.print(yawOffset, 2); Serial.println("°");
  Serial.println("🔧 Using Simple Low-pass Filter (α=0.1)");
  
  return true;
}

void processMPUData() {
  mpu6050.update();
  
  // Calculate delta time (masih digunakan untuk debug info)
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  
  // Apply offset untuk set posisi awal sebagai 0°
  rawPitch = mpu6050.getAngleX() - pitchOffset;
  rawRoll = mpu6050.getAngleY() - rollOffset;
  rawYaw = mpu6050.getAngleZ() - yawOffset;
  
  gyroX = mpu6050.getGyroX();
  gyroY = mpu6050.getGyroY();
  gyroZ = mpu6050.getGyroZ();
  
  accelX = mpu6050.getAccX();
  accelY = mpu6050.getAccY();
  accelZ = mpu6050.getAccZ();
  
  // Simple Low-pass Filter (tidak butuh dt)
  filteredPitch = filterPitch.update(rawPitch);
  filteredRoll = filterRoll.update(rawRoll);
  filteredYaw = filterYaw.update(rawYaw);
}

void checkConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi disconnected, reconnecting...");
    setupWiFi();
  }
  
  if (!mqttClient.connected()) {
    mqttConnected = false;
    reconnectMQTT();
  } else {
    mqttConnected = true;
    mqttClient.loop();
  }
}

void printDebugInfo() {
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 5000) {
    Serial.println();
    Serial.println("=== SYSTEM STATUS ===");
    Serial.print("MPU6050: "); Serial.println(mpuInitialized ? "OK" : "FAIL");
    Serial.print("WiFi: "); Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.print("MQTT: "); Serial.println(mqttConnected ? "Connected" : "Disconnected");
    Serial.print("Packets Sent: "); Serial.println(packetCounter);
    Serial.print("RSSI: "); Serial.println(WiFi.RSSI());
    
    // Tampilkan data sensor terbaru
    Serial.println("--- SENSOR DATA ---");
    Serial.print("Raw Pitch: "); Serial.print(rawPitch, 2);
    Serial.print("° -> Filtered: "); Serial.print(filteredPitch, 2); Serial.println("°");
    Serial.print("Raw Roll:  "); Serial.print(rawRoll, 2);
    Serial.print("° -> Filtered: "); Serial.print(filteredRoll, 2); Serial.println("°");
    Serial.print("Raw Yaw:   "); Serial.print(rawYaw, 2);
    Serial.print("° -> Filtered: "); Serial.print(filteredYaw, 2); Serial.println("°");
    Serial.println("===================");
    lastDebug = millis();
  }
}

// ==================== MAIN SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println();
  Serial.println("==================================");
  Serial.println("   PASAK - MQTT PUBLISHER");
  Serial.println("   MPU6050 Auto-Leveling System");
  Serial.println("   SIMPLE FILTER Version");
  Serial.println("==================================");
  
  // Initialize MPU6050
  mpuInitialized = initializeMPU6050();
  if (!mpuInitialized) {
    Serial.println("❌ CRITICAL: MPU6050 initialization failed!");
    while(1) {
      delay(1000);
    }
  }
  
  lastTime = micros();
  
  // Initialize WiFi & MQTT
  setupWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setBufferSize(512);
  
  Serial.println("✅ System Ready!");
  Serial.println("📤 Publishing to: " + String(TOPIC_PASAK_DATA));
  Serial.println("⏱  Send interval: " + String(SEND_INTERVAL) + "ms");
  Serial.println("🔧 Filter: Simple Low-pass (α=0.1)");
  Serial.println("==================================");
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentMillis = millis();
  
  // Process MPU data continuously
  if (mpuInitialized) {
    processMPUData();
  }
  
  // Check and maintain connections
  checkConnections();
  
  // Send data via MQTT at fixed interval
  if (currentMillis - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentMillis;
    if (mpuInitialized && mqttConnected) {
      publishSensorData();
    }
  }
  
  // Print debug info
  printDebugInfo();
  
  delay(10); // Small delay for stability
}
