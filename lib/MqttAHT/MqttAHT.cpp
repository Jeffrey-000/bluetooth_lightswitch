#include <MqttAHT.h>

MqttAHT::MqttAHT(const WifiInfo &wifi, const MqttInfo &mqtt, const int sda_pin, const int scl_pin)
    : MqttBase(wifi, mqtt), SDA_pin(sda_pin), SCl_pin(scl_pin)
{
}

void MqttAHT::begin()
{
    MqttBase::begin();
    Wire.begin(SDA_pin, SCl_pin);
    aht.begin(&Wire);
}

SensorData MqttAHT::readSensor()
{
    sensors_event_t humidityEvent, tempEvent;
    aht.getEvent(&humidityEvent, &tempEvent);
    return SensorData{tempEvent.temperature * 9 / 5 + 32, humidityEvent.relative_humidity};
}
