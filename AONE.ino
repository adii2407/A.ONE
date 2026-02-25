#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "MPU6050.h"
#include <TinyGPS++.h>
#include <SPI.h>
#include <LoRa.h>
#include "SPIFFS.h"
#include <math.h>

// ================= OBJECTS =================
MPU6050 mpu(0x69);
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// ================= LORA PINS =================
#define LORA_SS   15
#define LORA_RST  14
#define LORA_DIO0 26

// ================= BUZZER + LED =================
#define BUZZER_PIN 33
#define LED_PIN    32

String filename = "";
float prevTotalAcc = 1.0;
unsigned long lastBeepTime = 0;  

// ================= CREATE NEW FILE =================
void createNewFile() {
  int fileIndex = 1;
  while (true) {
    filename = "/LOG" + String(fileIndex) + ".csv";
    if (!SPIFFS.exists(filename)) {
      File file = SPIFFS.open(filename, FILE_WRITE);
      if (file) {
        file.println("AX,AY,AZ,TEMP,PRESS,ALT,LAT,LON,GPS_ALT,SATS");
        file.close();
      }
      break;
    }
    fileIndex++;
  }
  Serial.println("Created File: " + filename);
}

// ================= SETUP =================
void setup() {

  Serial.begin(115200);
  delay(1000);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  mpu.initialize();
  mpu.setSleepEnabled(false);
  Serial.println("MPU Initialized");

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found!");
    while (1);
  }
  Serial.println("BMP280 OK");

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed!");
    while (1);
  }
  Serial.println("SPIFFS Mounted");

  createNewFile();

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  Serial.println("LoRa Ready");

  Serial.println("========= System Ready =========");
}

// ================= LOOP =================
void loop() {

  // ===== GPS UPDATE =====
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // ===== MPU READ =====
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  float totalAcc = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  float motionChange = abs(totalAcc - prevTotalAcc);
  bool stationary = motionChange < 0.02;
  prevTotalAcc = totalAcc;

  // ===== STATIONARY BUZZER LOGIC =====
  if (stationary) {
    digitalWrite(LED_PIN, HIGH);

    if (millis() - lastBeepTime >= 2000) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);                  // 100ms beep
      digitalWrite(BUZZER_PIN, LOW);
      lastBeepTime = millis();
    }
  }
  else {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // ===== BMP =====
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0;
  float altitude = bmp.readAltitude(1013.25);

  // ===== GPS =====
  bool gpsValid = gps.location.isValid();
  float lat = gpsValid ? gps.location.lat() : 0;
  float lon = gpsValid ? gps.location.lng() : 0;
  float gpsAlt = gps.altitude.isValid() ? gps.altitude.meters() : 0;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

  // ===== SAVE TO SPIFFS =====
  File file = SPIFFS.open(filename, FILE_APPEND);
  bool memStatus = false;

  if (file) {
    file.print(ax_g); file.print(",");
    file.print(ay_g); file.print(",");
    file.print(az_g); file.print(",");
    file.print(temp); file.print(",");
    file.print(pressure); file.print(",");
    file.print(altitude); file.print(",");
    file.print(lat,6); file.print(",");
    file.print(lon,6); file.print(",");
    file.print(gpsAlt); file.print(",");
    file.println(sats);
    file.close();
    memStatus = true;
  }

  // ===== LoRa =====
  String packet = "A1," + String(temp) + "," + String(lat,6) + "," + String(lon,6);

  LoRa.beginPacket();
  LoRa.print(packet);
  int loraStatus = LoRa.endPacket();

  // ===== SERIAL DEBUG =====
  Serial.println("------ LOOP DATA ------");

  Serial.print("Temp: "); Serial.print(temp);
  Serial.print("  Alt: "); Serial.println(altitude);

  Serial.print("GPS Valid: "); Serial.println(gpsValid);
  Serial.print("LAT: "); Serial.print(lat,6);
  Serial.print(" LON: "); Serial.println(lon,6);
  Serial.print("SATS: "); Serial.println(sats);

  Serial.print("Motion: ");
  Serial.println(stationary ? "STATIONARY" : "MOVING");

  Serial.print("Memory: ");
  Serial.println(memStatus ? "Saved OK" : "Save Failed");

  Serial.print("LoRa Status: ");
  Serial.println(loraStatus == 1 ? "SENT ✅" : "FAILED ❌");

  Serial.println("-----------------------\n");

  delay(2000);
}
