#pragma once
#include <WifiManager.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <sys/time.h>
using namespace std;
struct MqttInfo
{
    std::string server;
    int port;
    std::string topic;
    std::string clientName;

    MqttInfo(const std::string &s, int p, const std::string &t, const std::string &c = "randomClient")
        : server(s), port(p), topic(t), clientName(c)
    {
    }
};

struct SensorData
{
    float temperature;
    float humidity;
};

class MqttBase
{
private:
    const WifiInfo wifiInfo;
    const MqttInfo mqttInfo;

public:
    WifiManager wifiManager;
    PubSubClient mqttClient;
    MqttBase(const WifiInfo &wifi, const MqttInfo &mqtt);
    virtual void begin();
    virtual void loop();
    virtual SensorData readSensor() = 0;
    static float CtoF(float c);
    bool publishToTopic(SensorData data);
    void reconnectToMqttServer();
};
