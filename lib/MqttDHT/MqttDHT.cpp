#include <MqttDHT.h>

MqttDHT::MqttDHT(const WifiInfo &wifi, const MqttInfo &mqtt, const int dhtpin, const int dhttype)
    : MqttBase(wifi, mqtt), dht(dhtpin, dhttype), lastSensorRead(0)
{
}

void MqttDHT::begin()
{
    MqttBase::begin();
    dht.begin();
}

SensorData MqttDHT::readSensor()
{
    unsigned long timeSinceLastRead = millis() - lastSensorRead;
    if (timeSinceLastRead < MIN_TIME_BETWEEN_SENSOR_DATA)
    {
        delay(MIN_TIME_BETWEEN_SENSOR_DATA - timeSinceLastRead);
    }
    lastSensorRead = millis();
    float t = dht.readTemperature(true);
    float h = dht.readHumidity();
    return SensorData{t, h};
}
