
#ifndef MQTTBASE_H
#define MQTTBASE_H

#include <WifiInfo.h>
#include <MqttInfo.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <sys/time.h>
using namespace std;


struct SensorData {
    float temperature;
    float humidity;
};

class MqttBase
{
private:
    WiFiClient wifiClient;
    PubSubClient mqttClient;
    const WifiInfo wifiInfo;
    const MqttInfo mqttInfo;

public:
    MqttBase(const WifiInfo &wifi, const MqttInfo &mqtt);
    virtual void begin();
    virtual void loop();
    //virtual SensorData readSensor();
    static float CtoF(float c);
    void setupWifi();
    void syncTime();
    void publishToTopic(SensorData data);
    void reconnectToMqttServer();
};

#endif