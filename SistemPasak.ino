#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

// ==================== MQTT CONFIGURATION ====================
const char* ssid = "";
const char* password = "";
const char* mqtt_server = ""; // atau domain
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ==================== MQTT TOPICS ====================
const char* TOPIC_PASAK_DATA = "pasak/sensor_data";  // Pasak publish ke sini

// ==================== PIN I2C untuk ESP32-C3 ====================
#define I2C_SDA 8
#define I2C_SCL 9

// ==================== MPU6050 OBJECT ====================
MPU6050 mpu6050(Wire);

// ==================== KALMAN FILTER CLASS ====================
class KalmanFilter {
public:
  KalmanFilter() {
    Q_angle = 0.001;
    Q_bias = 0.003;
    R_measure = 0.01;
    
    angle = 0.0;
    bias = 0.0;
    P[0][0] = 0.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 0.0;
  }
  
  double update(double newAngle, double newRate, double dt) {
    // Prediction step
    rate = newRate - bias;
    angle += dt * rate;
    
    // Update error covariance matrix
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    
    // Calculate Kalman gain
    double S = P[0][0] + R_measure;
    double K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    
    // Calculate innovation
    double y = newAngle - angle;
    
    // Update estimate
    angle += K[0] * y;
    bias += K[1] * y;
    
    // Update error covariance
    double P00_temp = P[0][0];
    double P01_temp = P[0][1];
    
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
    
    return angle;
  }
  
  void reset(double newAngle) {
    angle = newAngle;
    bias = 0.0;
    P[0][0] = 0.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 0.0;
  }
  
private:
  double Q_angle, Q_bias, R_measure;
  double angle, bias, rate;
  double P[2][2];
};

// ==================== GLOBAL VARIABLES ====================
KalmanFilter kalmanPitch, kalmanRoll, kalmanYaw;
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
      Serial.println("  JSON: " + String(jsonBuffer));
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
  
  // Reset Kalman filter
  kalmanPitch.reset(0);
  kalmanRoll.reset(0);
  kalmanYaw.reset(0);
  
  Serial.println("✅ Calibration and Leveling completed!");
  Serial.print("📐 Offset - Pitch: "); Serial.print(pitchOffset, 2);
  Serial.print("°, Roll: "); Serial.print(rollOffset, 2);
  Serial.print("°, Yaw: "); Serial.print(yawOffset, 2); Serial.println("°");
  
  return true;
}

void processMPUData() {
  mpu6050.update();
  
  // Calculate delta time
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  if (dt > 0.1) dt = 0.01; // Limit maximum dt
  
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
  
  // Kalman Filter
  filteredPitch = kalmanPitch.update(rawPitch, gyroY, dt);
  filteredRoll = kalmanRoll.update(rawRoll, gyroX, dt);
  filteredYaw = kalmanYaw.update(rawYaw, gyroZ, dt);
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
    Serial.print("Pitch: "); Serial.print(filteredPitch, 2); Serial.println("°");
    Serial.print("Roll:  "); Serial.print(filteredRoll, 2); Serial.println("°");
    Serial.print("Yaw:   "); Serial.print(filteredYaw, 2); Serial.println("°");
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
  Serial.println("   NO ArduinoJson Version");
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
  mqttClient.setBufferSize(512); // Increase buffer for JSON data
  
  Serial.println("✅ System Ready!");
  Serial.println("📤 Publishing to: " + String(TOPIC_PASAK_DATA));
  Serial.println("⏱  Send interval: " + String(SEND_INTERVAL) + "ms");
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
