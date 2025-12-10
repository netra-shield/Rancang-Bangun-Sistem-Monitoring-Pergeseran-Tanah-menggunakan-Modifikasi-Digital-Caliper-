#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==================== MQTT CONFIGURATION ====================
const char* ssid = "";
const char* password = "";
const char* mqtt_server = "";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ==================== MQTT TOPICS ====================
const char* TOPIC_PASAK_SUB = "pasak/sensor_data";
const char* TOPIC_DISPLACEMENT = "soil_monitoring/displacement";
const char* TOPIC_SOIL_MOISTURE = "soil_monitoring/soil_moisture";
const char* TOPIC_PITCH = "soil_monitoring/pitch";
const char* TOPIC_ROLL = "soil_monitoring/roll";
const char* TOPIC_YAW = "soil_monitoring/yaw";
const char* TOPIC_STATUS = "soil_monitoring/status";

// ==================== PIN DEFINITIONS ====================
#define CALIPER_CLOCK_PIN D0
#define CALIPER_DATA_PIN  D3
#define SOIL_POWER_PIN D4
#define SOIL_SENSOR_PIN A0
#define SD_CS_PIN D8

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// RTC
RTC_DS3231 rtc;

// ==================== DATA STRUCTURES ====================
struct SensorData {
  float pitch = 0.0;
  float roll = 0.0;
  float yaw = 0.0;
  float accelX = 0.0;
  float accelY = 0.0;
  float accelZ = 0.0;
  float gyroX = 0.0;
  float gyroY = 0.0;
  float gyroZ = 0.0;
  uint32_t packetCount = 0;
  unsigned long lastUpdate = 0;
  bool newData = false;
};

struct LocalSensorData {
  float caliperValue = 0.0;
  float soilMoisture = 0.0;
  float soilVoltage = 0.0;
  int soilADC = 0;
  bool soilSensorActive = false;
  unsigned long lastSoilCycle = 0;
  bool samplingDone = false;
  int sampleCount = 0;
  long sampleSum = 0;
  unsigned long lastSampleTime = 0;
};

SensorData pasakData;
LocalSensorData localData;

// ==================== SOIL MOISTURE CONFIGURATION ====================
const unsigned long SOIL_ACTIVE_TIME = 3000;  // aktif 3 detik
const unsigned long SOIL_IDLE_TIME = 7000;    // mati 7 detik
const int SOIL_NUM_SAMPLES = 30;              // jumlah sample tetap
const unsigned long SOIL_SAMPLE_INTERVAL = 100; // Interval antar sample (3000ms / 30 = 100ms)

// ==================== KALMAN FILTER FOR SOIL MOISTURE ====================
struct KalmanFilter {
  float Q = 2.0;
  float R = 8.0;
  float X = 0.0;
  float P = 1.0;
  bool initialized = false;
};

KalmanFilter soilKalman;

// ==================== CALIBRATION DATA ====================
const int NUM_POINTS = 6;
float calVolt[NUM_POINTS] = {0.973, 0.925, 0.275, 0.278, 0.221, 0.188};
float calMoist[NUM_POINTS] = {0, 20, 40, 60, 80, 100};

// ==================== ENHANCED LOGGING SYSTEM VARIABLES ====================
String currentLogFileName = "";
bool logFileCreated = false;
unsigned long lastSDWrite = 0;
const unsigned long SD_WRITE_INTERVAL = 1000; // Reduced to 1 second for smoother logging

// SD Card Buffer untuk write yang lebih smooth
String sdBuffer = "";
const int SD_BUFFER_MAX_SIZE = 10; // Max entries sebelum flush ke SD (dikurangi agar lebih responsif)
int sdBufferCount = 0;

// SD Card Status
bool sdCardReady = false;
bool sdWriting = false;
unsigned long lastSDStatusCheck = 0;
const unsigned long SD_STATUS_CHECK_INTERVAL = 10000; // Check SD status every 10 seconds

// ==================== TIMING CONFIGURATION ====================
const unsigned long DISPLAY_UPDATE_INTERVAL = 500;
const unsigned long MQTT_PUBLISH_INTERVAL = 500;
const unsigned long CONNECTION_CHECK_INTERVAL = 30000;

// ==================== GLOBAL VARIABLES ====================
unsigned long lastDisplayUpdate = 0;
unsigned long lastMQTTPublish = 0;
unsigned long lastConnectionCheck = 0;
unsigned long lastCaliperDebug = 0;

bool rtcReady = false;
bool mqttConnected = false;
unsigned long lastPasakMessage = 0;
uint32_t receivedPackets = 0;

// SD Card Statistics
unsigned long sdWriteCount = 0;
unsigned long sdErrorCount = 0;
unsigned long lastSDError = 0;

// SD Activity Indicator
bool sdActivityIndicator = false;
unsigned long sdActivityStartTime = 0;
const unsigned long SD_ACTIVITY_DISPLAY_TIME = 300; // Tampil 300ms saat writing

// ==================== DIGITAL CALIPER VARIABLES ====================
float caliperResult = 0.0;
bool caliperNewData = false;
int i;
int sign;
long value;
unsigned long tempmicros;

// ==================== MQTT CALLBACK ====================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("📥 Received: " + message);
  
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.println("❌ JSON parse error: " + String(error.c_str()));
    return;
  }
  
  // Update data dari pasak
  pasakData.pitch = doc["pitch"];
  pasakData.roll = doc["roll"];
  pasakData.yaw = doc["yaw"];
  pasakData.accelX = doc["ax"];
  pasakData.accelY = doc["ay"];
  pasakData.accelZ = doc["az"];
  pasakData.gyroX = doc["gx"];
  pasakData.gyroY = doc["gy"];
  pasakData.gyroZ = doc["gz"];
  pasakData.packetCount = doc["packet"];
  pasakData.lastUpdate = millis();
  pasakData.newData = true;
  
  receivedPackets++;
  lastPasakMessage = millis();
}

// ==================== MQTT FUNCTIONS ====================
void setupWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < 10000) {
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
    
    String clientId = "AlatUtama-";
    clientId += String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("✅ MQTT connected");
      mqttConnected = true;
      
      // Subscribe ke topic pasak
      if (mqttClient.subscribe(TOPIC_PASAK_SUB)) {
        Serial.println("✅ Subscribed to: " + String(TOPIC_PASAK_SUB));
      } else {
        Serial.println("❌ Failed to subscribe");
      }
      
    } else {
      Serial.print("❌ MQTT connection failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      mqttConnected = false;
      delay(5000);
    }
  }
}

void publishMonitoringData() {
  if (!mqttConnected) return;
  
  // Publish displacement
  mqttClient.publish(TOPIC_DISPLACEMENT, String(localData.caliperValue, 2).c_str());
  
  // Publish soil moisture
  mqttClient.publish(TOPIC_SOIL_MOISTURE, String(localData.soilMoisture, 1).c_str());
  
  // Publish orientation data dari pasak
  mqttClient.publish(TOPIC_PITCH, String(pasakData.pitch, 2).c_str());
  mqttClient.publish(TOPIC_ROLL, String(pasakData.roll, 2).c_str());
  mqttClient.publish(TOPIC_YAW, String(pasakData.yaw, 2).c_str());
  
  // Publish status
  String status = "{\"packets\":" + String(receivedPackets) + 
                  ",\"rssi\":" + String(WiFi.RSSI()) + 
                  ",\"sd_writes\":" + String(sdWriteCount) +
                  ",\"sd_errors\":" + String(sdErrorCount) + "}";
  mqttClient.publish(TOPIC_STATUS, status.c_str());
  
  lastMQTTPublish = millis();
}

// ==================== SOIL MOISTURE FUNCTIONS ====================
float kalmanUpdate(KalmanFilter &kf, float measurement) {
  if (!kf.initialized) {
    kf.X = measurement;
    kf.P = 1.0;
    kf.initialized = true;
  }
  kf.P += kf.Q;
  float K = kf.P / (kf.P + kf.R);
  kf.X += K * (measurement - kf.X);
  kf.P *= (1 - K);
  return kf.X;
}

float mapMoisture(float volt) {
  // batas atas-bawah
  if (volt >= calVolt[0]) return calMoist[0];
  if (volt <= calVolt[NUM_POINTS - 1]) return calMoist[NUM_POINTS - 1];

  for (int i = 0; i < NUM_POINTS - 1; i++) {
    if (volt <= calVolt[i] && volt >= calVolt[i + 1]) {
      float m = (calMoist[i + 1] - calMoist[i]) / (calVolt[i + 1] - calVolt[i]);
      return calMoist[i] + m * (volt - calVolt[i]);
    }
  }
  return 0;
}

void startSoilSampling() {
  digitalWrite(SOIL_POWER_PIN, HIGH);
  localData.soilSensorActive = true;
  localData.samplingDone = false;
  localData.sampleCount = 0;
  localData.sampleSum = 0;
  localData.lastSampleTime = millis();
  
  Serial.println("\n[Soil Probe ON - Sampling started]");
}

void processSoilSampling() {
  unsigned long now = millis();
  
  if (now - localData.lastSampleTime >= SOIL_SAMPLE_INTERVAL) {
    localData.lastSampleTime = now;
    
    // Baca sensor dan proses Kalman filter
    int adcRaw = analogRead(SOIL_SENSOR_PIN);
    float filtered = kalmanUpdate(soilKalman, adcRaw);
    localData.sampleSum += filtered;
    localData.sampleCount++;
    
    // Debug info setiap 10 samples
    if (localData.sampleCount % 10 == 0) {
      Serial.print("ADC[");
      Serial.print(localData.sampleCount);
      Serial.print("] = ");
      Serial.print(adcRaw);
      Serial.print(" | Filtered: ");
      Serial.println(filtered, 2);
    }
    
    // Cek apakah sampling sudah selesai
    if (localData.sampleCount >= SOIL_NUM_SAMPLES) {
      // Hitung rata-rata
      float avgADC = (float)localData.sampleSum / SOIL_NUM_SAMPLES;
      float avgV1 = (avgADC / 1023.0) * 1.0;  // 1V reference internal ESP8266
      float moistPercent = mapMoisture(avgV1);
      
      // Update data
      localData.soilADC = avgADC;
      localData.soilVoltage = avgV1;
      localData.soilMoisture = moistPercent;
      localData.samplingDone = true;
      
      Serial.println("[Soil Sampling selesai]");
      Serial.print("ADC avg: "); Serial.print(avgADC, 2);
      Serial.print(" | Volt(1V): "); Serial.print(avgV1, 3);
      Serial.print(" | Moisture: "); Serial.print(moistPercent, 1);
      Serial.println(" %");
      
      // Matikan probe
      digitalWrite(SOIL_POWER_PIN, LOW);
      localData.soilSensorActive = false;
      localData.lastSoilCycle = millis();
      Serial.println("[Soil Probe OFF]");
    }
  }
}

void processSoilSensor() {
  unsigned long now = millis();
  
  if (!localData.soilSensorActive) {
    // Jika probe tidak aktif dan sudah melewati idle time, mulai sampling
    if (now - localData.lastSoilCycle >= SOIL_IDLE_TIME) {
      startSoilSampling();
    }
  } else {
    // Jika probe aktif, proses sampling (non-blocking)
    processSoilSampling();
  }
}

// ==================== ENHANCED LOGGING SYSTEM ====================
String getTimestamp() {
  if (rtcReady) {
    DateTime now = rtc.now();
    char buffer[20];
    sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d", 
            now.year(), now.month(), now.day(), 
            now.hour(), now.minute(), now.second());
    return String(buffer);
  } else {
    return String(millis() / 1000);
  }
}

String createLogFileName() {
  if (rtcReady) {
    DateTime now = rtc.now();
    char buffer[20];
    sprintf(buffer, "%04d%02d%02d_%02d%02d%02d.csv", 
            now.year(), now.month(), now.day(), 
            now.hour(), now.minute(), now.second());
    return String(buffer);
  } else {
    return "soil_data.csv"; // Fallback name
  }
}

bool createLogFile() {
  if (currentLogFileName == "") {
    currentLogFileName = createLogFileName();
  }
  
  File dataFile = SD.open(currentLogFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println("Waktu,Displacement(mm),Soil_Moisture(%),Pitch(deg),Roll(deg),Yaw(deg),aX(g),aY(g),aZ(g),gX(dps),gY(dps),gZ(dps),Packet_Count");
    dataFile.close();
    
    logFileCreated = true;
    
    Serial.println("📁 Log file created: " + currentLogFileName);
    Serial.println("💾 Enhanced logging with buffer system");
    return true;
  } else {
    Serial.println("❌ Failed to create log file");
    return false;
  }
}

void addDataToBuffer() {
  if (!sdCardReady) return;
  
  String dataString = getTimestamp() + ",";
  dataString += String(localData.caliperValue, 2) + ",";
  dataString += String(localData.soilMoisture, 1) + ",";
  dataString += String(pasakData.pitch, 2) + ",";
  dataString += String(pasakData.roll, 2) + ",";
  dataString += String(pasakData.yaw, 2) + ",";
  dataString += String(pasakData.accelX, 2) + ",";
  dataString += String(pasakData.accelY, 2) + ",";
  dataString += String(pasakData.accelZ, 2) + ",";
  dataString += String(pasakData.gyroX, 2) + ",";
  dataString += String(pasakData.gyroY, 2) + ",";
  dataString += String(pasakData.gyroZ, 2) + ",";
  dataString += String(pasakData.packetCount);
  
  sdBuffer += dataString + "\n";
  sdBufferCount++;
}

bool flushBufferToSD() {
  if (sdBuffer.length() == 0) return true;
  
  // Aktifkan indikator writing
  sdWriting = true;
  sdActivityIndicator = true;
  sdActivityStartTime = millis();
  
  File dataFile = SD.open(currentLogFileName, FILE_WRITE);
  if (dataFile) {
    int bytesWritten = dataFile.print(sdBuffer);
    dataFile.close();
    
    sdWriting = false;
    
    if (bytesWritten > 0) {
      sdWriteCount += sdBufferCount;
      sdBuffer = "";
      sdBufferCount = 0;
      Serial.println("💾 Data flushed to SD. Total: " + String(sdWriteCount));
      return true;
    }
  }
  
  sdWriting = false;
  sdErrorCount++;
  lastSDError = millis();
  Serial.println("❌ Failed to write buffer to SD");
  return false;
}

void writeDataToLog() {
  if (!sdCardReady || !logFileCreated) return;
  
  // Tambah data ke buffer
  addDataToBuffer();
  
  // Flush buffer jika sudah penuh atau interval waktu tercapai
  if (sdBufferCount >= SD_BUFFER_MAX_SIZE) {
    flushBufferToSD();
  }
}

void checkSDCardStatus() {
  if (millis() - lastSDStatusCheck >= SD_STATUS_CHECK_INTERVAL) {
    lastSDStatusCheck = millis();
    
    if (!SD.begin(SD_CS_PIN)) {
      Serial.println("❌ SD Card reinit failed");
      sdCardReady = false;
    } else {
      sdCardReady = true;
    }
  }
}

// ==================== DIGITAL CALIPER FUNCTIONS ====================
void readCaliper() {
  while (digitalRead(CALIPER_CLOCK_PIN) == HIGH) {}
  tempmicros = micros();
  while (digitalRead(CALIPER_CLOCK_PIN) == LOW) {}
  
  if ((micros() - tempmicros) > 500) {
    decodeCaliper();
  }
}

void decodeCaliper() {
  sign = 1;
  value = 0;
  
  for (i = 0; i < 23; i++) {
    while (digitalRead(CALIPER_CLOCK_PIN) == HIGH) { }
    while (digitalRead(CALIPER_CLOCK_PIN) == LOW) {}
    
    if (digitalRead(CALIPER_DATA_PIN) == LOW) {
      if (i < 20) {
        value |= 1 << i;
      }
      if (i == 20) {
        sign = -1;
      }
    }
  }

  caliperResult = (value * sign) / 100.00;
  caliperNewData = true;
  localData.caliperValue = caliperResult;
}

// ==================== ENHANCED DISPLAY FUNCTIONS ====================
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  // Header dengan status koneksi dan SD
  display.print(mqttConnected ? "M:T" : "M:N");
  display.print(" SD:");
  
  // Tampilkan status SD dengan indikator aktivitas
  if (sdActivityIndicator) {
    display.print("WR"); // Indikator sedang menulis
  } else if (sdCardReady) {
    display.print("Y");
  } else {
    display.print("N");
  }
  
  display.print(" Pkt:");
  display.println(receivedPackets);
  bool pasakOnline = (millis() - lastPasakMessage < 5000);

  display.print("P:");
  display.print(pasakOnline ? "Y" : "N");
  display.print(" Buf:");
  display.print(sdBufferCount);
  display.print(" SD:");
  display.print(sdWriteCount);
  display.print("W ");
  display.print(sdErrorCount);
  display.println("E");

  display.println("-------------------");
  //---------------------------------------------------------------

  display.setTextSize(2);
  display.print(localData.caliperValue, 2);
  display.println("mm");
  display.setTextSize(1);
  display.print("Soil:");
  display.print(localData.soilMoisture, 1);
  display.println("%");
  display.print("P:");
  display.print(pasakData.pitch, 1);
  display.print(" R:");
  display.print(pasakData.roll, 1);
  display.print("Y:");
  display.println(pasakData.yaw, 1);
  
  
  
  
  display.display();
  
  // Update indikator aktivitas SD
  if (sdActivityIndicator && (millis() - sdActivityStartTime > SD_ACTIVITY_DISPLAY_TIME)) {
    sdActivityIndicator = false;
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n🚀 Alat Utama - Enhanced Logging System");
  Serial.println("💾 Buffer-based SD logging with OLED indicator");
  
  // Initialize GPIO
  pinMode(CALIPER_CLOCK_PIN, INPUT);
  pinMode(CALIPER_DATA_PIN, INPUT);
  pinMode(SOIL_POWER_PIN, OUTPUT);
  digitalWrite(SOIL_POWER_PIN, LOW);  // Pastikan mati saat boot
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("❌ OLED init failed!");
  } else {
    Serial.println("✅ OLED initialized");
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Alat Utama");
    display.println("Enhanced Logging");
    display.println("OLED Indicators");
    display.display();
  }
  
  // Initialize RTC
  if (rtc.begin()) {
    rtcReady = true;
    Serial.println("✅ RTC initialized");
  } else {
    Serial.println("❌ RTC not found");
  }
  
  // Initialize SD Card
  if (SD.begin(SD_CS_PIN)) {
    sdCardReady = true;
    Serial.println("✅ SD Card initialized");
    // Buat file log sekali saja saat startup
    createLogFile();
  } else {
    Serial.println("❌ SD Card init failed");
  }
  
  // Initialize WiFi & MQTT
  setupWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  
  Serial.println("✅ Enhanced System Ready");
  Serial.println("📥 Subscribing to: " + String(TOPIC_PASAK_SUB));
  Serial.println("💾 Buffer logging: " + String(SD_BUFFER_MAX_SIZE) + " entries buffer");
  Serial.println("📊 OLED SD activity indicators");
}

// ==================== CONNECTION MONITORING ====================
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

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentMillis = millis();
  
  // === POLLING DIGITAL CALIPER ===
  readCaliper();
  
  // Process soil sensor dengan algoritma non-blocking
  processSoilSensor();
  
  // Check connections
  checkConnections();
  
  // Check SD card status periodically
  checkSDCardStatus();
  
  // Publish monitoring data
  if (currentMillis - lastMQTTPublish >= MQTT_PUBLISH_INTERVAL) {
    lastMQTTPublish = currentMillis;
    publishMonitoringData();
  }
  
  // Write to SD card buffer setiap 1 detik
  if (currentMillis - lastSDWrite >= SD_WRITE_INTERVAL) {
    lastSDWrite = currentMillis;
    writeDataToLog();
  }
  
  // Update display
  if (currentMillis - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = currentMillis;
    updateDisplay();
  }
  
  // Debug info
  if (currentMillis - lastCaliperDebug >= 30000) {
    Serial.println("🔍 Enhanced System Status:");
    Serial.print("  Displacement: "); Serial.print(localData.caliperValue, 2); Serial.println(" mm");
    Serial.print("  Soil: "); Serial.print(localData.soilMoisture, 1); Serial.println(" %");
    Serial.print("  Soil ADC: "); Serial.print(localData.soilADC); Serial.print(" | Volt: "); Serial.println(localData.soilVoltage, 3);
    Serial.print("  Pitch: "); Serial.print(pasakData.pitch, 1); Serial.println(" deg");
    Serial.print("  MQTT: "); Serial.println(mqttConnected ? "Connected" : "Disconnected");
    Serial.print("  Pasak Packets: "); Serial.println(receivedPackets);
    Serial.print("  SD Writes: "); Serial.print(sdWriteCount); Serial.print(" | Errors: "); Serial.println(sdErrorCount);
    Serial.print("  Buffer: "); Serial.print(sdBufferCount); Serial.print("/"); Serial.println(SD_BUFFER_MAX_SIZE);
    Serial.print("  Log File: "); Serial.println(currentLogFileName);
    lastCaliperDebug = currentMillis;
  }
  
  delay(10);
}
