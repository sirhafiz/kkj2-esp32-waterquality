#define BLYNK_TEMPLATE_ID "TMPL6zG03a79_"
#define BLYNK_TEMPLATE_NAME "TDS"
#define BLYNK_AUTH_TOKEN "7vXt_5Gv3Z3vxiRMR6TjOKyC17qDM3UB"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // Sesuaikan alamat I2C jika berbeza
#define TDS_PIN 35  // Pin analog GPIO 35

namespace sensor {
  unsigned int tds = 0;
  const int tdsThreshold = 600;  // Ambang untuk hidroponik (diubah ke 600 ppm)
  bool alertSent = false;
  bool alertSentStable = false;  // Bendera untuk notifikasi stabil
}

void setup() {
  Serial.begin(115200);
  Serial.println("Memulakan sistem TDS IoT untuk Hidroponik...");

  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Sistem TDS IoT");
  lcd.setCursor(0, 1);
  lcd.print("Memulakan...");
  delay(2000);

  pinMode(TDS_PIN, INPUT);

  // WiFiManager untuk sambungan WiFi
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(180);
  if (!wifiManager.autoConnect("ESP32-TDS-AP-Hafiz")) {
    Serial.println("Gagal sambung WiFi!");
    lcd.clear();
    lcd.print("WiFi Gagal!");
    delay(5000);
    ESP.restart();
  } else {
    Serial.println("WiFi tersambung!");
    lcd.clear();
    lcd.print("WiFi OK!");
    delay(2000);
  }

  // Persediaan Blynk
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();
  if (Blynk.connected()) {
    Serial.println("Blynk tersambung!");
    lcd.clear();
    lcd.print("Blynk OK!");
    delay(2000);
  } else {
    Serial.println("Blynk gagal sambung!");
  }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();
  } else {
    Serial.println("WiFi terputus! Cuba sambung semula...");
    Blynk.disconnect();
    Blynk.connect();
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
  float voltage = (float)sensorValue * 3.3 / 4095.0;  // ADC 12-bit, sensor 3.3V

  Serial.print("Raw Analog (Avg): ");
  Serial.print(sensorValue);
  Serial.print(" | Max Raw: ");
  Serial.print(maxReading);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" | Valid Readings: ");
  Serial.println(validReadings);

  // Kira TDS berdasarkan voltan
  float offset = 0.14;
  float ec = (voltage * 1.0) - offset;  // EC disederhanakan untuk pengiraan TDS
  if (ec < 0) ec = 0;
  sensor::tds = (133.42 * pow(ec, 3) - 255.86 * ec * ec + 857.39 * ec) * 0.5;

  Serial.print("TDS: ");
  Serial.println(sensor::tds);

  // Papar pada LCD: TDS di baris pertama, kualiti di baris kedua
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TDS: ");
  lcd.print(sensor::tds);
  lcd.print(" ppm");
  lcd.setCursor(0, 1);
  if (sensor::tds < 50) lcd.print("Sangat Baik");
  else if (sensor::tds < 300) lcd.print("Baik");
  else if (sensor::tds < 600) lcd.print("Sederhana");
  else lcd.print("Buruk");

  // Hantar ke Blynk
  Blynk.virtualWrite(V0, sensor::tds);

  // Logik notifikasi
  if (sensor::tds > sensor::tdsThreshold && !sensor::alertSent) {
    Blynk.logEvent("poor_water_quality", "⚠️ Kualiti air rendah! TDS: " + String(sensor::tds) + " ppm");
    sensor::alertSent = true;
    sensor::alertSentStable = false;
  } else if (sensor::tds <= sensor::tdsThreshold && !sensor::alertSentStable) {
    Blynk.logEvent("stable_water_quality", "✅ Kualiti air kembali stabil! TDS: " + String(sensor::tds) + " ppm");
    sensor::alertSentStable = true;
    sensor::alertSent = false;
  }
}