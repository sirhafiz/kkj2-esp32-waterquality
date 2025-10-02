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
#define SOIL_PIN 2  // Pin analog GPIO 2 untuk sensor kelembapan tanah

// Inisialisasi OneWire dan DallasTemperature
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Inisialisasi WiFiManager
WiFiManager wifiManager;

namespace sensor {
  float ec = 0;
  unsigned int tds = 0;
  float temperature = 0;  // Menyimpan bacaan suhu
  unsigned int soilMoisture = 0;  // Menyimpan bacaan kelembapan tanah (0-100%)
  float ecCalibration = 1.0;
  const int tdsThresholdMin = 300;  // Ambang minimum untuk akuaponik
  const int tdsThresholdMax = 1200; // Ambang maksimum untuk akuaponik
  const float tempThresholdMin = 20.0;  // Ambang minimum suhu (°C)
  const float tempThresholdMax = 30.0;  // Ambang maksimum suhu (°C)
  const int soilThresholdMin = 30;  // Ambang minimum kelembapan tanah (%)
  const int soilThresholdMax = 70;  // Ambang maksimum kelembapan tanah (%)
  bool alertSentLow = false;
  bool alertSentHigh = false;
  bool alertSentNormal = false;
  bool alertSentTempLow = false;  // Bendera untuk notifikasi suhu rendah
  bool alertSentTempHigh = false; // Bendera untuk notifikasi suhu tinggi
  bool alertSentTempNormal = false; // Bendera untuk notifikasi suhu normal
  bool alertSentSoilLow = false;  // Bendera untuk notifikasi kelembapan tanah rendah
  bool alertSentSoilHigh = false; // Bendera untuk notifikasi kelembapan tanah tinggi
  bool alertSentSoilNormal = false; // Bendera untuk notifikasi kelembapan tanah normal
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

  pinMode(TDS_PIN, INPUT);
  pinMode(SOIL_PIN, INPUT);  // Inisialisasi pin untuk sensor kelembapan tanah

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

  readSensors();
  delay(2000);
}

void readSensors() {
  const int numReadings = 50;  // Ambil purata 50 bacaan
  long sumTds = 0;
  long sumSoil = 0;
  int validReadings = 0;
  int maxTdsReading = 0;
  int maxSoilReading = 0;
  
  // Membaca TDS
  for (int i = 0; i < numReadings; i++) {
    int tdsReading = analogRead(TDS_PIN);
    int soilReading = analogRead(SOIL_PIN);
    if (tdsReading > 0) {
      sumTds += tdsReading;
      validReadings++;
      if (tdsReading > maxTdsReading) maxTdsReading = tdsReading;
    }
    if (soilReading > 0) {
      sumSoil += soilReading;
      if (soilReading > maxSoilReading) maxSoilReading = soilReading;
    }
    delay(5);
  }
  
  // Proses bacaan TDS
  int tdsSensorValue = (validReadings > 0) ? (sumTds / validReadings) : 0;
  float rawEc = (float)tdsSensorValue * 3.3 / 4095.0;  // ADC 12-bit, sensor 3.3V

  // Proses bacaan kelembapan tanah
  int soilSensorValue = (validReadings > 0) ? (sumSoil / validReadings) : 0;
  // Map bacaan analog (0-4095) ke peratusan (0-100%)
  // Nilai 4095 = kering (0%), 0 = basah (100%)
  sensor::soilMoisture = map(soilSensorValue, 4095, 0, 0, 100);
  if (sensor::soilMoisture < 0) sensor::soilMoisture = 0;
  if (sensor::soilMoisture > 100) sensor::soilMoisture = 100;

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

  // Log bacaan ke Serial Monitor
  Serial.print("Raw Analog TDS (Avg): ");
  Serial.print(tdsSensorValue);
  Serial.print(" | Max TDS: ");
  Serial.print(maxTdsReading);
  Serial.print(" | Voltage: ");
  Serial.print(rawEc, 3);
  Serial.print(" | Valid Readings: ");
  Serial.print(validReadings);
  Serial.print(" | Temperature: ");
  Serial.print(sensor::temperature, 1);
  Serial.print(" °C | Soil Moisture: ");
  Serial.print(sensor::soilMoisture);
  Serial.println(" %");

  // Proses TDS dan EC
  float offset = 0.14;
  sensor::ec = (rawEc * sensor::ecCalibration) - offset;
  if (sensor::ec < 0) sensor::ec = 0;
  sensor::tds = (133.42 * pow(sensor::ec, 3) - 255.86 * sensor::ec * sensor::ec + 857.39 * sensor::ec) * 0.5;

  Serial.print("TDS: ");
  Serial.print(sensor::tds);
  Serial.print(" ppm | EC: ");
  Serial.print(sensor::ec, 2);
  Serial.print(" | Temperature: ");
  Serial.print(sensor::temperature, 1);
  Serial.print(" °C | Soil Moisture: ");
  Serial.print(sensor::soilMoisture);
  Serial.println(" %");

  // Papar pada LCD (bergilir antara TDS, suhu, dan kelembapan tanah)
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TDS:");
  lcd.print(sensor::tds);
  lcd.print(" T:");
  lcd.print(sensor::temperature, 1);
  lcd.setCursor(0, 1);
  lcd.print("EC:");
  lcd.print(sensor::ec, 2);
  lcd.print(" S:");
  lcd.print(sensor::soilMoisture);
  lcd.print("%");

  // Hantar ke Blynk
  if (Blynk.connected()) {
    Blynk.virtualWrite(V0, sensor::tds);
    Blynk.virtualWrite(V1, sensor::ec);
    Blynk.virtualWrite(V2, sensor::temperature);
    Blynk.virtualWrite(V3, sensor::soilMoisture);  // Hantar kelembapan tanah ke pin virtual V3
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

  // Logik notifikasi kelembapan tanah
  if (sensor::soilMoisture < sensor::soilThresholdMin && !sensor::alertSentSoilLow) {
    Blynk.logEvent("low_soil", "⚠️ Kelembapan tanah terlalu rendah! Kelembapan: " + String(sensor::soilMoisture) + " %");
    sensor::alertSentSoilLow = true;
    sensor::alertSentSoilNormal = false;
  } else if (sensor::soilMoisture > sensor::soilThresholdMax && !sensor::alertSentSoilHigh) {
    Blynk.logEvent("high_soil", "⚠️ Kelembapan tanah terlalu tinggi! Kelembapan: " + String(sensor::soilMoisture) + " %");
    sensor::alertSentSoilHigh = true;
    sensor::alertSentSoilNormal = false;
  } else if (sensor::soilMoisture >= sensor::soilThresholdMin && sensor::soilMoisture <= sensor::soilThresholdMax && !sensor::alertSentSoilNormal) {
    Blynk.logEvent("normal_soil", "✅ Kelembapan tanah kembali normal! Kelembapan: " + String(sensor::soilMoisture) + " %");
    sensor::alertSentSoilNormal = true;
    sensor::alertSentSoilLow = false;
    sensor::alertSentSoilHigh = false;
  }
}