#include "SECRETS.h"
#include <MqttDHT.h>

WifiInfo wifiInfo{
    SSID,
    PASSWORD};
MqttInfo mqttInfo{
    SERVER, PORT, "sensors/temperature/dht22/bedroom"};

MqttDHT mqttDHT(wifiInfo, mqttInfo, 5, DHT22);

unsigned int lastMessage = 0;
unsigned int frequency = 10000; // 10 seconds

//----------------------------------------------------
void setup()
{
  Serial.begin(115200);
  mqttDHT.begin();
}

void loop()
{
  unsigned long now = millis();
  mqttDHT.loop();
  if (now - lastMessage > frequency)
  {
    SensorData data = mqttDHT.readSensor();
    mqttDHT.publishToTopic(data);
    lastMessage = millis();
  }
}
