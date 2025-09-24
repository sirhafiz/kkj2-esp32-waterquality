#define BLYNK_TEMPLATE_ID "TMPL6zG03a79_"
#define BLYNK_TEMPLATE_NAME "TDS"
#define BLYNK_AUTH_TOKEN "7vXt_5Gv3Z3vxiRMR6TjOKyC17qDM3UB"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// WiFi credentials (ganti dengan SSID dan password anda)
const char* ssid = "Redmi13";       // Contoh: "WiFi Student"
const char* password = "ha.fiz123";

LiquidCrystal_I2C lcd(0x27, 16, 2); // Sesuaikan alamat I2C jika berbeza
#define TDS_PIN 35  // Pin analog GPIO 35
#define ONE_WIRE_BUS 15  // Pin D15 untuk DS18B20

// Inisialisasi OneWire dan DallasTemperature
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

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
  bool alertSentTempLow = false;  // Bendera untuk notifikasi suhu rendah
  bool alertSentTempHigh = false; // Bendera untuk notifikasi suhu tinggi
  bool alertSentTempNormal = false; // Bendera untuk notifikasi suhu normal
}

void setup() {
  Serial.begin(115200);
  Serial.println("Memulakan sistem TDS dan Suhu IoT untuk Akuaponik...");

  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Sistem Akuaponik");
  lcd.setCursor(0, 1);
  lcd.print("Memulakan...");
  delay(2000);

  pinMode(TDS_PIN, INPUT);

  // Inisialisasi sensor DS18B20
  sensors.begin();

  // Sambung ke WiFi
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sambung WiFi...");
  WiFi.begin(ssid, password);
  int wifiTimeout = 20; // Timeout 20 saat
  unsigned long startAttemptTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout * 1000) {
    delay(500);
    Serial.print(".");
    lcd.setCursor(0, 1);
    lcd.print("Menunggu...");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi tersambung! SSID: " + String(ssid));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi OK!");
    lcd.setCursor(0, 1);
    lcd.print(ssid);
    delay(2000);
  } else {
    Serial.println("\nGagal sambung WiFi! Mulakan semula...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Gagal!");
    lcd.setCursor(0, 1);
    lcd.print("Restart...");
    delay(5000);
    ESP.restart();
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

void loop() {
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    Blynk.run();
  } else {
    Serial.println("WiFi atau Blynk terputus! Cuba sambung semula...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Koneksi Terputus");
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, password);
      int wifiTimeout = 10; // Timeout 10 saat untuk reconnect
      unsigned long startAttemptTime = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout * 1000) {
        delay(500);
        Serial.print(".");
        lcd.setCursor(0, 1);
        lcd.print("Sambung WiFi...");
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
    } else {
      Serial.println("Gagal sambung WiFi!");
      lcd.setCursor(0, 1);
      lcd.print("WiFi Gagal!");
      delay(2000);
    }
  }

  readTdsQuick();
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

  // Papar pada LCD: TDS dan suhu di baris pertama, EC dan kualiti di baris kedua
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TDS:");
  lcd.print(sensor::tds);
  lcd.print(" T:");
  lcd.print(sensor::temperature, 1);
  lcd.setCursor(0, 1);
  lcd.print("EC:");
  lcd.print(sensor::ec, 2);
  lcd.print(" ");
  if (sensor::tds < 300) lcd.print("Rendah");
  else if (sensor::tds <= 1200) lcd.print("Baik");
  else lcd.print("Tinggi");

  // Hantar ke Blynk
  if (Blynk.connected()) {
    Blynk.virtualWrite(V0, sensor::tds);
    Blynk.virtualWrite(V1, sensor::ec);
    Blynk.virtualWrite(V2, sensor::temperature);  // Hantar suhu ke pin virtual V2
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