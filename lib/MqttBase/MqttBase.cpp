#include <MqttBase.h>

MqttBase::MqttBase(const WifiInfo &wifi, const MqttInfo &mqtt)
    : wifiClient(),
      mqttClient(wifiClient), wifiInfo(wifi), mqttInfo(mqtt)
{
}

void MqttBase::begin()
{
    setupWifi();
    syncTime();
    mqttClient.setServer(mqttInfo.server.c_str(), mqttInfo.port);
}

void MqttBase::loop()
{
    if (!mqttClient.connected())
    {
        reconnectToMqttServer();
    }
    mqttClient.loop();
}

void MqttBase::setupWifi()
{
    delay(10);
    Serial.println();
    Serial.print("Connecting to WiFi: ");
    Serial.println(wifiInfo.ssid.c_str());

    WiFi.begin(wifiInfo.ssid.c_str(), wifiInfo.password.c_str());

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}
void MqttBase::syncTime()
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
    if(data.humidity == 0 || data.temperature == 0 || data.temperature == 32) {
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