#include <Arduino.h>
#include <NimBLEDevice.h>
#include <BLE_CONSTS.h>
#include <../lib/Servo/Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <time.h>
#include <sys/time.h>
#include "SECRETS.h"

void setup_wifi();
void reconnect();
void syncTime();
float CtoF(float c);
void readFromSensor(unsigned long now);
void sendMessage(unsigned long now);

#define SERVICE "BAAD"
#define CHR "FOOD" // dont work when not food for some reason

bool connectToServer();
void notifyCB(NimBLERemoteCharacteristic *pRemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);
bool handleConnection(NimBLEClient *pClient);
void initBLE();

static uint32_t scanTimeMs = 5000; /** scan time in milliseconds, 0 = scan forever */
static bool readyToConnect = false;
static bool isConnected = false;
const int servoPin = 13;

// DHT settings
const uint8_t DHTPIN = 4; // GPIO where the DHT sensor is connected
const uint8_t DHTTYPE = DHT22;
// https://www.adafruit.com/product/386?srsltid=AfmBOoq0uIDvW8eU0y9S7mg77y8f04Icmpm7jYoAt8YaKJMwxaD57tUQ
DHT dht(DHTPIN, DHTTYPE);

// Clients & Servers
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Timing
const int INTERVAL = 5000; // Publish every 5 seconds. max is every 2 seconds
unsigned long lastMsgTime = 0;
unsigned long lastSensorRead = 0;

// Globals
float temp = -1;     // only good to zero c
float humidity = -1; // humidity in %, lowest it reads is 20%

static const NimBLEAdvertisedDevice *advDevice;
class ClientCallbacks : public NimBLEClientCallbacks
{
    void onConnect(NimBLEClient *pClient) override
    {
        isConnected = true;
    }

    void onDisconnect(NimBLEClient *pClient, int reason) override
    {
        Serial.printf("%s Disconnected, reason = %d - Starting scan\n", pClient->getPeerAddress().toString().c_str(), reason);
        isConnected = false;
        NimBLEDevice::getScan()->start(scanTimeMs, false, true);
    }
} clientCallbacks;

/** Define a class to handle the callbacks when scan events are received */
class ScanCallbacks : public NimBLEScanCallbacks
{
    void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override
    {
        // Serial.printf("Advertised Device found: %s\n", advertisedDevice->toString().c_str());
        Serial.printf(".");
        if (advertisedDevice->isAdvertisingService(NimBLEUUID(SERVICE)))
        {
            Serial.printf("Found Our Service\n");
            /** stop scan before connecting */
            NimBLEDevice::getScan()->stop();
            /** Save the device reference in a global for the client to use*/
            advDevice = advertisedDevice;
            /** Ready to connect now */
            readyToConnect = true;
        }
    }

    void onScanEnd(const NimBLEScanResults &scanResults, int reason) override
    {
        Serial.printf("starting scan again\n");
        if (!isConnected && !readyToConnect)
        {
            NimBLEDevice::getScan()->start(scanTimeMs, false, true);
        }
    }
} scanCallbacks;

Servo servo(servoPin);

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    dht.begin();
    servo.begin();
    servo.setAngle(0);
    initBLE();
    setup_wifi();
    syncTime();
    mqttClient.setServer(SERVER, PORT);
}

void loop()
{
    /** Loop here until we find a device we want to connect to */
    // delay(10);

    if (readyToConnect)
    {
        readyToConnect = false;
        /** Found a device we want to connect to, do it now */
        if (connectToServer())
        {
            Serial.printf("Success! we should now be getting notifications, scanning for more!\n");
        }
        else
        {
            Serial.printf("Failed to connect, starting scan\n");
        }
        // NimBLEDevice::getScan()->start(scanTimeMs, false, true);
    }
    if (!mqttClient.connected())
    {
        reconnect();
    }
    mqttClient.loop();

    unsigned long now = millis();
    readFromSensor(now);
    sendMessage(now);
}

void notifyCB(NimBLERemoteCharacteristic *pRemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    // std::string str = (isNotify == true) ? "Notification" : "Indication";
    // str += " from ";
    // str += pRemoteCharacteristic->getClient()->getPeerAddress().toString();
    // str += ": Service = " + pRemoteCharacteristic->getRemoteService()->getUUID().toString();
    // str += ", Characteristic = " + pRemoteCharacteristic->getUUID().toString();
    // str += ", Value = " + std::string((char *)pData, length);
    // Serial.printf("%s\n", str.c_str());
    std::string dataString = std::string((char *)pData, length);
    int angle = 0;
    try
    {
        angle = std::stoi(dataString);
    }
    catch (...)
    {
        return;
    }
    servo.setAngle(angle);
    if (angle < 50)
    {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
        digitalWrite(BUILTIN_LED, LOW);
    }
}

bool handleConnection(NimBLEClient *pClient)
{
    NimBLERemoteService *pSvc = nullptr;
    NimBLERemoteCharacteristic *pChr = nullptr;
    pSvc = pClient->getService(SERVICE);
    if (pSvc)
    {
        pChr = pSvc->getCharacteristic(CHR);
    }
    else
    {
        Serial.printf("service not found\n");
    }

    // subscribe to lights.state
    if (pChr)
    {

        if (pChr->canNotify())
        {

            if (!pChr->subscribe(true, notifyCB))
            {
                pClient->disconnect();
                return false;
            }
        }
        else if (pChr->canIndicate())
        {
            /** Send false as first argument to subscribe to indications instead of notifications */
            if (!pChr->subscribe(false, notifyCB))
            {
                pClient->disconnect();
                return false;
            }
        }
    }
    else
    {
        Serial.printf("characteristic not found.\n");
        // pClient->disconnect();

        // Serial.printf("%s service not found.\n", pSvc->getUUID());
        // return false;
    }
    return true;
}

/** Handles the provisioning of clients and connects / interfaces with the server */
bool connectToServer()
{
    NimBLEClient *pClient = nullptr;

    /** Check if we have a client we should reuse first **/
    if (NimBLEDevice::getCreatedClientCount())
    {
        /**
         *  Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if (pClient)
        {
            if (!pClient->connect(advDevice, false))
            {
                Serial.printf("Reconnect failed\n");
                return false;
            }
            Serial.printf("Reconnected client\n");
        }
        else
        {
            /**
             *  We don't already have a client that knows this device,
             *  check for a client that is disconnected that we can use.
             */
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if (!pClient)
    {
        if (NimBLEDevice::getCreatedClientCount() >= NIMBLE_MAX_CONNECTIONS)
        {
            Serial.printf("Max clients reached - no more connections available\n");
            return false;
        }

        pClient = NimBLEDevice::createClient();

        Serial.printf("New client created\n");

        pClient->setClientCallbacks(&clientCallbacks, false);
        /**
         *  Set initial connection parameters:
         *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
         *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
         *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 150 * 10ms = 1500ms timeout
         */
        pClient->setConnectionParams(12, 12, 0, 150);

        /** Set how long we are willing to wait for the connection to complete (milliseconds), default is 30000. */
        pClient->setConnectTimeout(5 * 1000);

        if (!pClient->connect(advDevice))
        {
            /** Created a client but failed to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(pClient);
            Serial.printf("Failed to connect, deleted client\n");
            return false;
        }
    }

    if (!pClient->isConnected())
    {
        if (!pClient->connect(advDevice))
        {
            Serial.printf("Failed to connect\n");
            return false;
        }
    }

    Serial.printf("Connected to: %s RSSI: %d\n", pClient->getPeerAddress().toString().c_str(), pClient->getRssi());
    /** Now we can read/write/subscribe the characteristics of the services we are interested in */
    handleConnection(pClient);
    return true;
}

void initBLE()
{
    Serial.printf("Starting NimBLE Client\n");
    NimBLEDevice::init("NimBLE-Client");
    NimBLEDevice::setPower(3);
    NimBLEScan *pScan = NimBLEDevice::getScan();

    pScan->setScanCallbacks(&scanCallbacks, false);
    pScan->setInterval(100);
    pScan->setWindow(100);
    pScan->setActiveScan(false);

    pScan->start(scanTimeMs, false, true);
    Serial.printf("Scanning for peripherals\n");
}

//----------------------------------------------------
float CtoF(float c)
{
    return c * 9 / 5 + 32;
}

void readFromSensor(unsigned long now)
{
    if (now - lastSensorRead > INTERVAL)
    {
        lastSensorRead = now;
        temp = CtoF(dht.readTemperature());
        humidity = dht.readHumidity();
    }
}

void sendMessage(unsigned long now)
{
    if (now - lastMsgTime > INTERVAL)
    {
        lastMsgTime = now;
        if (isnan(temp) || isnan(humidity))
        {
            Serial.println("Failed to read from DHT sensor");
            return;
        }
        char jsonBuffer[100];
        snprintf(jsonBuffer, sizeof(jsonBuffer),
                 "{\"temperature\": %.2f, \"humidity\": %.2f, \"time\": %ld}",
                 temp, humidity, time(nullptr));

        Serial.println("Publishing JSON:");
        Serial.println(jsonBuffer);

        mqttClient.publish(TOPIC, jsonBuffer);
    }
}

void setup_wifi()
{
    delay(10);
    Serial.println();
    Serial.print("Connecting to WiFi: ");
    Serial.println(SSID);

    WiFi.begin(SSID, PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void reconnect()
{
    // Loop until reconnected
    while (!mqttClient.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (mqttClient.connect("ESP32Client"))
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

void syncTime()
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
