
#ifndef MQTTAHT_H
#define MQTTAHT_H
#include <MqttBase.h>
#include <Adafruit_AHTX0.h>




class MqttAHT : public MqttBase
{
private:
    Adafruit_AHTX0 aht;
    const int SDA_pin;
    const int SCl_pin;

public:
    MqttAHT(const WifiInfo &wifi, const MqttInfo &mqtt, const int sda_pin, const int scl_pin);
    virtual void begin() override;
    SensorData readSensor();
};
#endif