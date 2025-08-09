#include <Wire.h>
#include <Adafruit_AHTX0.h>

#define SDA_PIN 5
#define SCL_PIN 4

Adafruit_AHTX0 aht;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); 

  while (!Serial) delay(10); // Wait for Serial

  Serial.println("Initializing AHT30...");

  if (!aht.begin(&Wire)) {
    Serial.println("Failed to find AHT30 chip");
    while (1) delay(10);
  }
  Serial.println("AHT30 found!");
}

void loop() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp); // Get temperature & humidity

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" %");

  delay(2000);
}
