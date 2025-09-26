#define BLYNK_TEMPLATE_ID "TMPL6zG03a79_"
#define BLYNK_TEMPLATE_NAME "Temperature"
#define BLYNK_AUTH_TOKEN "7vXt_5Gv3Z3vxiRMR6TjOKyC17qDM3UB"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 15  // Pin D15 for DS18B20

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address for LCD
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

namespace sensor {
  float temperature = 0.0;
  const float tempThresholdMin = 20.0;  // Minimum temperature threshold (°C)
  const float tempThresholdMax = 30.0;  // Maximum temperature threshold (°C)
  bool alertSentLow = false;
  bool alertSentHigh = false;
  bool alertSentNormal = false;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting DS18B20 IoT System for Aquaponics...");

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Aquaponics System");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(2000);

  // Initialize DS18B20
  sensors.begin();

  // WiFiManager for WiFi connection
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(180);
  if (!wifiManager.autoConnect("ESP32-Temp-AP")) {
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

  // Blynk
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

  readTemperature();
  delay(2000);
}

void readTemperature() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: Error");
    return;
  }

  sensor::temperature = tempC;

  Serial.print("Temperature: ");
  Serial.print(sensor::temperature);
  Serial.println(" °C");

  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(sensor::temperature, 1);
  lcd.print(" C");
  lcd.setCursor(0, 1);
  if (sensor::temperature < sensor::tempThresholdMin) lcd.print("Low");
  else if (sensor::temperature <= sensor::tempThresholdMax) lcd.print("Good");
  else lcd.print("High");

  // Send to Blynk
  Blynk.virtualWrite(V0, sensor::temperature);

  // Notification logic
  if (sensor::temperature < sensor::tempThresholdMin && !sensor::alertSentLow) {
    Blynk.logEvent("low_temp", "⚠️ Temperature too low! Temp: " + String(sensor::temperature) + " °C");
    sensor::alertSentLow = true;
    sensor::alertSentNormal = false;
  } else if (sensor::temperature > sensor::tempThresholdMax && !sensor::alertSentHigh) {
    Blynk.logEvent("high_temp", "⚠️ Temperature too high! Temp: " + String(sensor::temperature) + " °C");
    sensor::alertSentHigh = true;
    sensor::alertSentNormal = false;
  } else if (sensor::temperature >= sensor::tempThresholdMin && sensor::temperature <= sensor::tempThresholdMax && !sensor::alertSentNormal) {
    Blynk.logEvent("normal_temp", "✅ Temperature normal! Temp: " + String(sensor::temperature) + " °C");
    sensor::alertSentNormal = true;
    sensor::alertSentLow = false;
    sensor::alertSentHigh = false;
  }
}