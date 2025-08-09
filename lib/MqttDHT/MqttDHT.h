
#ifndef MQTTDHT_H
#define MQTTDHT_H
#include <MqttBase.h>
#include <DHT.h>
#define MIN_TIME_BETWEEN_SENSOR_DATA 2000 //2 seconds


class MqttDHT : public MqttBase
{
private:
    DHT dht;
    unsigned long lastSensorRead;

public:
    MqttDHT(const WifiInfo &wifi, const MqttInfo &mqtt, const int dhtpin, const int dhttype);
    virtual void begin() override;
    SensorData readSensor();
};
#endif