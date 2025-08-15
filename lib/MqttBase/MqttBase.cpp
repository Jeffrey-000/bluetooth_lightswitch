#include <MqttBase.h>

MqttBase::MqttBase(const WifiInfo &wifi, const MqttInfo &mqtt)
    : wifiManager(wifi),
      mqttClient(wifiManager.getClient()), wifiInfo(wifi), mqttInfo(mqtt)
{
}

void MqttBase::begin()
{
    wifiManager.begin();
    wifiManager.syncTime();
    mqttClient.setServer(mqttInfo.server.c_str(), mqttInfo.port);
}

void MqttBase::loop()
{
    wifiManager.loop();
    if (!mqttClient.connected())
    {
        reconnectToMqttServer();
    }
    mqttClient.loop();
}

void MqttBase::reconnectToMqttServer()
{
    String clientId = String(mqttInfo.clientName.c_str()) + String(random(0xffff), HEX);

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

bool MqttBase::publishToTopic(SensorData data)
{
    if (data.humidity == 0 || data.temperature == 0 || data.temperature == 32)
    {
        return false;
    }
    char jsonBuffer[100];
    snprintf(jsonBuffer, sizeof(jsonBuffer),
             "{\"temperature\": %.2f, \"humidity\": %.2f, \"time\": %ld}",
             data.temperature, data.humidity, time(nullptr));

    Serial.println("Publishing JSON:");
    Serial.println(jsonBuffer);

    mqttClient.publish(mqttInfo.topic.c_str(), jsonBuffer);
    return true;
}
