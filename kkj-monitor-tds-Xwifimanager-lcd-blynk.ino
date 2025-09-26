#define BLYNK_TEMPLATE_ID "TMPL6zG03a79_"
#define BLYNK_TEMPLATE_NAME "TDS"
#define BLYNK_AUTH_TOKEN "7vXt_5Gv3Z3vxiRMR6TjOKyC17qDM3UB"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>

// Ganti dengan kredensial WiFi anda
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

LiquidCrystal_I2C lcd(0x27, 16, 2); // Sesuaikan alamat I2C jika berbeza
#define TDS_PIN 35  // Pin analog GPIO 35

namespace sensor {
  float ec = 0;
  unsigned int tds = 0;
  float ecCalibration = 1.0;
  const int tdsThreshold = 300;  // Ambang untuk hydroponik
  bool alertSent = false;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Memulakan sistem TDS IoT...");

  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Sistem TDS IoT");
  lcd.setCursor(0, 1);
  lcd.print("Memulakan...");
  delay(2000);

  pinMode(TDS_PIN, INPUT);

  // Sambungan WiFi langsung
  WiFi.begin(ssid, password);
  lcd.clear();
  lcd.print("Sambung WiFi...");
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi tersambung!");
    lcd.clear();
    lcd.print("WiFi OK!");
    delay(2000);
  } else {
    Serial.println("\nGagal sambung WiFi!");
    lcd.clear();
    lcd.print("WiFi Gagal!");
    delay(5000);
    ESP.restart();
  }

  // Blynk
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
    WiFi.reconnect();
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
  float rawEc = (float)sensorValue * 3.3 / 4095.0;  // ADC 12-bit, sensor 3.3V

  Serial.print("Raw Analog (Avg): ");
  Serial.print(sensorValue);
  Serial.print(" | Max Raw: ");
  Serial.print(maxReading);
  Serial.print(" | Voltage: ");
  Serial.print(rawEc, 3);
  Serial.print(" | Valid Readings: ");
  Serial.println(validReadings);

  float offset = 0.14;
  sensor::ec = (rawEc * sensor::ecCalibration) - offset;
  if (sensor::ec < 0) sensor::ec = 0;

  sensor::tds = (133.42 * pow(sensor::ec, 3) - 255.86 * sensor::ec * sensor::ec + 857.39 * sensor::ec) * 0.5;

  Serial.print("TDS: ");
  Serial.println(sensor::tds);
  Serial.print("EC: ");
  Serial.println(sensor::ec, 2);

  // Papar pada LCD: TDS di baris pertama, EC dan kualiti di baris kedua
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TDS: ");
  lcd.print(sensor::tds);
  lcd.print(" ppm");
  lcd.setCursor(0, 1);
  lcd.print("EC:");
  lcd.print(sensor::ec, 2);
  lcd.print(" ");
  if (sensor::tds < 50) lcd.print("SgtBaik");
  else if (sensor::tds < 300) lcd.print("Baik");
  else if (sensor::tds < 600) lcd.print("Sederhana");
  else lcd.print("Buruk");

  // Hantar ke Blynk
  Blynk.virtualWrite(V0, sensor::tds);
  Blynk.virtualWrite(V1, sensor::ec);

  // Logik notifikasi
  if (sensor::tds > sensor::tdsThreshold && !sensor::alertSent) {
    Blynk.logEvent("poor_water_quality", "⚠️ Kualiti air rendah! TDS: " + String(sensor::tds) + " ppm");
    sensor::alertSent = true;
  } else if (sensor::tds <= sensor::tdsThreshold && sensor::alertSent) {
    sensor::alertSent = false;
  }
}