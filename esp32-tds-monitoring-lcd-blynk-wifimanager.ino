#define BLYNK_TEMPLATE_ID "TMPL6zG03a79_"
#define BLYNK_TEMPLATE_NAME "TDS"
#define BLYNK_AUTH_TOKEN "7vXt_5Gv3Z3vxiRMR6TjOKyC17qDM3UB"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // Adjust I2C address if different
#define TDS_PIN 35  // Analog pin GPIO 35

namespace sensor {
  unsigned int tds = 0;
  const int tdsThreshold = 300;  // Threshold for hydroponics
  bool alertSent = false;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting TDS IoT system...");

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("TDS IoT System");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(2000);

  pinMode(TDS_PIN, INPUT);

  // WiFiManager for WiFi connection
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(180);
  if (!wifiManager.autoConnect("ESP32-TDS-AP-Hafiz")) {
    Serial.println("WiFi connection failed!");
    lcd.clear();
    lcd.print("WiFi Failed!");
    delay(5000);
    ESP.restart();
  } else {
    Serial.println("WiFi connected!");
    lcd.clear();
    lcd.print("WiFi OK!");
    delay(2000);
  }

  // Blynk setup
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();
  if (Blynk.connected()) {
    Serial.println("Blynk connected!");
    lcd.clear();
    lcd.print("Blynk OK!");
    delay(2000);
  } else {
    Serial.println("Blynk connection failed!");
  }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();
  } else {
    Serial.println("WiFi disconnected! Attempting to reconnect...");
    Blynk.disconnect();
    Blynk.connect();
  }

  readTdsQuick();
  delay(2000);
}

void readTdsQuick() {
  const int numReadings = 50;  // Average of 50 readings
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
  float voltage = (float)sensorValue * 3.3 / 4095.0;  // ADC 12-bit, 3.3V sensor

  Serial.print("Raw Analog (Avg): ");
  Serial.print(sensorValue);
  Serial.print(" | Max Raw: ");
  Serial.print(maxReading);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" | Valid Readings: ");
  Serial.println(validReadings);

  // Calculate TDS based on voltage
  float offset = 0.14;
  float ec = (voltage * 1.0) - offset;  // Simplified EC for TDS calculation
  if (ec < 0) ec = 0;
  sensor::tds = (133.42 * pow(ec, 3) - 255.86 * ec * ec + 857.39 * ec) * 0.5;

  Serial.print("TDS: ");
  Serial.println(sensor::tds);

  // Display on LCD: TDS on first line, quality on second line
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TDS: ");
  lcd.print(sensor::tds);
  lcd.print(" ppm");
  lcd.setCursor(0, 1);
  if (sensor::tds < 50) lcd.print("Very Good");
  else if (sensor::tds < 300) lcd.print("Good");
  else if (sensor::tds < 600) lcd.print("Moderate");
  else lcd.print("Poor");

  // Send to Blynk
  Blynk.virtualWrite(V0, sensor::tds);

  // Notification logic
  if (sensor::tds > sensor::tdsThreshold && !sensor::alertSent) {
    Blynk.logEvent("poor_water_quality", "⚠️ Poor water quality! TDS: " + String(sensor::tds) + " ppm");
    sensor::alertSent = true;
  } else if (sensor::tds <= sensor::tdsThreshold && sensor::alertSent) {
    sensor::alertSent = false;
  }
}