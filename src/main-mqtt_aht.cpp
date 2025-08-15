#include "SECRETS.h"
#include <MqttAHT.h>

WifiInfo wifiInfo{
    SSID,
    PASSWORD};
MqttInfo mqttInfo{
    SERVER, PORT, "sensors/temperature/aht30/bedroom"};

MqttAHT mqttAHT(wifiInfo, mqttInfo, 5, 4);

unsigned int lastMessage = 0;
unsigned int frequency = 10000; // 10 seconds

//----------------------------------------------------
void setup()
{
  Serial.begin(115200);
  mqttAHT.begin();
}

void loop()
{
  unsigned long now = millis();
  mqttAHT.loop();
  if (now - lastMessage > frequency)
  {
    SensorData data = mqttAHT.readSensor();
    mqttAHT.publishToTopic(data);
    lastMessage = millis();
  }
}
