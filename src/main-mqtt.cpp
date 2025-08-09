#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <time.h>
#include <sys/time.h>
#include "SECRETS.h"

void setup_wifi();
void reconnect();
void syncTime();
float CtoF(float c);
void readFromSensor(unsigned long now);
void sendMessage(unsigned long now);



// DHT settings
const uint8_t DHTPIN = 4; // GPIO where the DHT sensor is connected
const uint8_t DHTTYPE = DHT22;
// https://www.adafruit.com/product/386?srsltid=AfmBOoq0uIDvW8eU0y9S7mg77y8f04Icmpm7jYoAt8YaKJMwxaD57tUQ
DHT dht(DHTPIN, DHTTYPE);

// Clients & Servers
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Timing
const int INTERVAL = 5000; // Publish every 5 seconds. max is every 2 seconds
unsigned long lastMsgTime = 0;
unsigned long lastSensorRead = 0;

// Globals
float temp = -1;     // only good to zero c
float humidity = -1; // humidity in %, lowest it reads is 20%

//----------------------------------------------------
void setup()
{
  Serial.begin(115200);
  dht.begin();
  setup_wifi();
  syncTime();
  mqttClient.setServer(SERVER, PORT);
}

void loop()
{
  if (!mqttClient.connected())
  {
    reconnect();
  }
  mqttClient.loop();

  unsigned long now = millis();
  readFromSensor(now);
  sendMessage(now);
}
//----------------------------------------------------
float CtoF(float c)
{
  return c * 9 / 5 + 32;
}

void readFromSensor(unsigned long now)
{
  if (now - lastSensorRead > INTERVAL)
  {
    lastSensorRead = now;
    temp = CtoF(dht.readTemperature());
    humidity = dht.readHumidity();
  }
}

void sendMessage(unsigned long now)
{
  if (now - lastMsgTime > INTERVAL)
  {
    lastMsgTime = now;
    if (isnan(temp) || isnan(humidity))
    {
      Serial.println("Failed to read from DHT sensor");
      return;
    }
    char jsonBuffer[100];
    snprintf(jsonBuffer, sizeof(jsonBuffer),
             "{\"temperature\": %.2f, \"humidity\": %.2f, \"time\": %ld}",
             temp, humidity, time(nullptr));

    Serial.println("Publishing JSON:");
    Serial.println(jsonBuffer);

    mqttClient.publish(TOPIC, jsonBuffer);
  }
}

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(SSID);

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  String clientId = "esp32-dht-" + String(random(0xffff), HEX);

  // Loop until reconnected
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str()))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void syncTime()
{
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("Waiting for NTP time sync...");
  while (time(nullptr) < 100000)
  {
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nTime synced!");
}

