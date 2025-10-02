#define BLYNK_TEMPLATE_ID "TMPL6zG03a79_"
#define BLYNK_TEMPLATE_NAME "TDS"
#define BLYNK_AUTH_TOKEN "7vXt_5Gv3Z3vxiRMR6TjOKyC17qDM3UB"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiManager.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Inisialisasi LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // Sesuaikan alamat I2C jika berbeza
#define TDS_PIN 35  // Pin analog GPIO 35 untuk sensor TDS
#define ONE_WIRE_BUS 15  // Pin D15 untuk DS18B20
#define SOIL_PIN 2  // Pin D2 untuk sensor kelembapan tanah
#define RELAY1_PIN 14  // Pin D14 untuk Relay 1
#define RELAY2_PIN 13  // Pin D13 untuk Relay 2

// Inisialisasi OneWire dan DallasTemperature
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Inisialisasi WiFiManager
WiFiManager wifiManager;

namespace sensor {
  float ec = 0;
  unsigned int tds = 0;
  float temperature = 0;  // Menyimpan bacaan suhu
  float ecCalibration = 1.0;
  const int tdsThresholdMin = 300;  // Ambang minimum untuk akuaponik
  const int tdsThresholdMax = 1200; // Ambang maksimum untuk akuaponik
  const float tempThresholdMin = 20.0;  // Ambang minimum suhu (°C)
  const float tempThresholdMax = 30.0;  // Ambang maksimum suhu (°C)
  bool alertSentLow = false;
  bool alertSentHigh = false;
  bool alertSentNormal = false;
  bool alertSentTempLow = false;
  bool alertSentTempHigh = false;
  bool alertSentTempNormal = false;
  int soilMoisture = 0;  // Menyimpan bacaan kelembapan tanah
  const int soilThresholdMin = 30;  // Ambang minimum kelembapan tanah (%)
  const int soilThresholdMax = 60;  // Ambang maksimum kelembapan tanah (%)
  bool alertSentSoilLow = false;
  bool alertSentSoilHigh = false;
  bool alertSentSoilNormal = false;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Memulakan sistem TDS, Suhu, dan Kelembapan Tanah IoT untuk Akuaponik...");

  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Sistem Akuaponik");
  lcd.setCursor(0, 1);
  lcd.print("Memulakan...");
  delay(2000);

  // Inisialisasi pin
  pinMode(TDS_PIN, INPUT);
  pinMode(SOIL_PIN, INPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, HIGH); // Relay OFF (aktif rendah)
  digitalWrite(RELAY2_PIN, HIGH); // Relay OFF (aktif rendah)

  // Inisialisasi sensor DS18B20
  sensors.begin();

  // Sambung ke WiFi menggunakan WiFiManager
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sambung WiFi...");
  
  // Konfigurasi WiFiManager
  wifiManager.setTimeout(180); // Timeout 180 saat untuk konfigurasi WiFi
  wifiManager.setAPCallback(configModeCallback); // Callback ketika masuk ke mode AP

  if (!wifiManager.autoConnect("Aquaponics_AP")) {
    Serial.println("Gagal sambung WiFi dan timeout! Mulakan semula...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Gagal!");
    lcd.setCursor(0, 1);
    lcd.print("Restart...");
    delay(5000);
    ESP.restart();
  } else {
    Serial.println("\nWiFi tersambung! SSID: " + WiFi.SSID());
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi OK!");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.SSID());
    delay(2000);
  }

  // Blynk
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();
  if (Blynk.connected()) {
    Serial.println("Blynk tersambung!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Blynk OK!");
    delay(2000);
  } else {
    Serial.println("Blynk gagal sambung!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Blynk Gagal!");
    delay(2000);
  }
}

// Callback ketika ESP32 masuk ke mode Access Point (AP)
void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Masuk mode konfigurasi WiFi!");
  Serial.println("Sila sambung ke AP: " + String(myWiFiManager->getConfigPortalSSID()));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sambung ke AP:");
  lcd.setCursor(0, 1);
  lcd.print(myWiFiManager->getConfigPortalSSID());
}

void loop() {
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    Blynk.run();
  } else {
    Serial.println("WiFi atau Blynk terputus! Cuba sambung semula...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Koneksi Terputus");
    if (WiFi.status() != WL_CONNECTED) {
      lcd.setCursor(0, 1);
      lcd.print("Sambung WiFi...");
      if (!wifiManager.autoConnect("Aquaponics_AP")) {
        Serial.println("Gagal sambung WiFi!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WiFi Gagal!");
        lcd.setCursor(0, 1);
        lcd.print("Restart...");
        delay(5000);
        ESP.restart();
      }
    }
    if (WiFi.status() == WL_CONNECTED) {
      Blynk.connect();
      if (Blynk.connected()) {
        Serial.println("WiFi dan Blynk tersambung semula!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Koneksi OK!");
        delay(2000);
      }
    }
  }

  readTdsQuick();
  readSoilMoisture();
  delay(2000);
}

void readTdsQuick() {
  const int numReadings = 50;  // Ambil purata 50 bacaan
  long sum = 0;
  int validReadings = 0;
  int maxReading = 0;
  for (int i = 0; i < numReadings; i++) {
    int reading = analogRead(TDS_PIN);
    if (reading > 0) {
      sum += reading;
      validReadings++;
      if (reading > maxReading) maxReading = reading;
    }
    delay(5);
  }
  int sensorValue = (validReadings > 0) ? (sum / validReadings) : 0;
  float rawEc = (float)sensorValue * 3.3 / 4095.0;  // ADC 12-bit, sensor 3.3V

  // Membaca suhu dari DS18B20
  sensors.requestTemperatures();
  sensor::temperature = sensors.getTempCByIndex(0);  // Suhu dalam Celsius

  // Validasi bacaan suhu
  if (sensor::temperature == -127.0) {
    Serial.println("Gagal membaca suhu dari DS18B20!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Suhu Error!");
    return;
  }

  Serial.print("Raw Analog (Avg): ");
  Serial.print(sensorValue);
  Serial.print(" | Max Raw: ");
  Serial.print(maxReading);
  Serial.print(" | Voltage: ");
  Serial.print(rawEc, 3);
  Serial.print(" | Valid Readings: ");
  Serial.print(validReadings);
  Serial.print(" | Temperature: ");
  Serial.print(sensor::temperature, 1);
  Serial.println(" °C");

  float offset = 0.14;
  sensor::ec = (rawEc * sensor::ecCalibration) - offset;
  if (sensor::ec < 0) sensor::ec = 0;

  sensor::tds = (133.42 * pow(sensor::ec, 3) - 255.86 * sensor::ec * sensor::ec + 857.39 * sensor::ec) * 0.5;

  Serial.print("TDS: ");
  Serial.println(sensor::tds);
  Serial.print("EC: ");
  Serial.println(sensor::ec, 2);
  Serial.print("Temperature: ");
  Serial.println(sensor::temperature, 1);

  // Hantar ke Blynk
  if (Blynk.connected()) {
    Blynk.virtualWrite(V0, sensor::tds);
    Blynk.virtualWrite(V1, sensor::ec);
    Blynk.virtualWrite(V2, sensor::temperature);
  }

  // Logik notifikasi TDS
  if (sensor::tds < sensor::tdsThresholdMin && !sensor::alertSentLow) {
    Blynk.logEvent("low_tds", "⚠️ TDS terlalu rendah! TDS: " + String(sensor::tds) + " ppm");
    sensor::alertSentLow = true;
    sensor::alertSentNormal = false;
  } else if (sensor::tds > sensor::tdsThresholdMax && !sensor::alertSentHigh) {
    Blynk.logEvent("high_tds", "⚠️ TDS terlalu tinggi! TDS: " + String(sensor::tds) + " ppm");
    sensor::alertSentHigh = true;
    sensor::alertSentNormal = false;
  } else if (sensor::tds >= sensor::tdsThresholdMin && sensor::tds <= sensor::tdsThresholdMax && !sensor::alertSentNormal) {
    Blynk.logEvent("normal_tds", "✅ TDS kembali normal! TDS: " + String(sensor::tds) + " ppm");
    sensor::alertSentNormal = true;
    sensor::alertSentLow = false;
    sensor::alertSentHigh = false;
  }

  // Logik notifikasi suhu
  if (sensor::temperature < sensor::tempThresholdMin && !sensor::alertSentTempLow) {
    Blynk.logEvent("low_temp", "⚠️ Suhu terlalu rendah! Suhu: " + String(sensor::temperature, 1) + " °C");
    sensor::alertSentTempLow = true;
    sensor::alertSentTempNormal = false;
  } else if (sensor::temperature > sensor::tempThresholdMax && !sensor::alertSentTempHigh) {
    Blynk.logEvent("high_temp", "⚠️ Suhu terlalu tinggi! Suhu: " + String(sensor::temperature, 1) + " °C");
    sensor::alertSentTempHigh = true;
    sensor::alertSentTempNormal = false;
  } else if (sensor::temperature >= sensor::tempThresholdMin && sensor::temperature <= sensor::tempThresholdMax && !sensor::alertSentTempNormal) {
    Blynk.logEvent("normal_temp", "✅ Suhu kembali normal! Suhu: " + String(sensor::temperature, 1) + " °C");
    sensor::alertSentTempNormal = true;
    sensor::alertSentTempLow = false;
    sensor::alertSentTempHigh = false;
  }
}

void readSoilMoisture() {
  const int numReadings = 10;  // Ambil purata 10 bacaan
  long sum = 0;
  int validReadings = 0;
  for (int i = 0; i < numReadings; i++) {
    int reading = analogRead(SOIL_PIN);
    if (reading > 0) {
      sum += reading;
      validReadings++;
    }
    delay(5);
  }
  int sensorValue = (validReadings > 0) ? (sum / validReadings) : 0;

  // Tukar bacaan analog kepada peratusan kelembapan (0% = kering, 100% = basah)
  sensor::soilMoisture = map(sensorValue, 4095, 0, 0, 100); // Terbalik kerana sensor kelembapan tanah biasanya rendah apabila basah

  Serial.print("Soil Moisture: ");
  Serial.print(sensor::soilMoisture);
  Serial.println(" %");

  // Kawal relay berdasarkan kelembapan tanah
  if (sensor::soilMoisture < sensor::soilThresholdMin) {
    digitalWrite(RELAY1_PIN, LOW); // Hidupkan Relay 1 (aktif rendah)
    digitalWrite(RELAY2_PIN, LOW); // Hidupkan Relay 2 (aktif rendah)
  } else {
    digitalWrite(RELAY1_PIN, HIGH); // Matikan Relay 1
    digitalWrite(RELAY2_PIN, HIGH); // Matikan Relay 2
  }

  // Papar pada LCD: TDS dan suhu di baris pertama, Kelembapan dan status di baris kedua
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TDS:");
  lcd.print(sensor::tds);
  lcd.print(" T:");
  lcd.print(sensor::temperature, 1);
  lcd.setCursor(0, 1);
  lcd.print("Soil:");
  lcd.print(sensor::soilMoisture);
  lcd.print("% ");
  if (sensor::soilMoisture < sensor::soilThresholdMin) lcd.print("Kering");
  else if (sensor::soilMoisture > sensor::soilThresholdMax) lcd.print("Basah");
  else lcd.print("Baik");

  // Hantar ke Blynk
  if (Blynk.connected()) {
    Blynk.virtualWrite(V3, sensor::soilMoisture);
  }

  // Logik notifikasi kelembapan tanah
  if (sensor::soilMoisture < sensor::soilThresholdMin && !sensor::alertSentSoilLow) {
    Blynk.logEvent("low_soil", "⚠️ Tanah terlalu kering! Kelembapan: " + String(sensor::soilMoisture) + " %");
    sensor::alertSentSoilLow = true;
    sensor::alertSentSoilNormal = false;
  } else if (sensor::soilMoisture > sensor::soilThresholdMax && !sensor::alertSentSoilHigh) {
    Blynk.logEvent("high_soil", "⚠️ Tanah terlalu basah! Kelembapan: " + String(sensor::soilMoisture) + " %");
    sensor::alertSentSoilHigh = true;
    sensor::alertSentSoilNormal = false;
  } else if (sensor::soilMoisture >= sensor::soilThresholdMin && sensor::soilMoisture <= sensor::soilThresholdMax && !sensor::alertSentSoilNormal) {
    Blynk.logEvent("normal_soil", "✅ Kelembapan tanah normal! Kelembapan: " + String(sensor::soilMoisture) + " %");
    sensor::alertSentSoilNormal = true;
    sensor::alertSentSoilLow = false;
    sensor::alertSentSoilHigh = false;
  }
}