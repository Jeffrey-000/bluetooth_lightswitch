#include <Arduino.h>
#include <NimBLEDevice.h>
#include <BLE_CONSTS.h>
#include <../lib/Servo/Servo.h>
#include "SECRETS.h"
#include <MqttAHT.h>

#define SERVICE "BAAD"
#define CHR "FOOD" // dont work when not food for some reason

bool connectToServer();
void notifyCB(NimBLERemoteCharacteristic *pRemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);
bool handleConnection(NimBLEClient *pClient);
void initBLE();

const int servoPin = 14;
const int SDA_pin = 26;
const int SCL_pin = 25;
const int button_pin = 12;


static uint32_t scanTimeMs = 5000; /** scan time in milliseconds, 0 = scan forever */
static bool readyToConnect = false;
static bool isConnected = false;
unsigned long lastButtonPress = 0;
unsigned int debounceTime = 500;
unsigned int lastMessage = 0;
unsigned int frequency = 10000; // 10 seconds


WifiInfo wifiInfo{
    SSID,
    PASSWORD};
MqttInfo mqttInfo{
    SERVER, PORT, "sensors/temperature/aht20/lightswitch"};

MqttAHT mqttAHT(wifiInfo, mqttInfo, SDA_pin, SCL_pin);


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
    mqttAHT.begin();
    initBLE();
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(button_pin, INPUT_PULLUP);
    servo.setUnpoweredIfIdle(true);
    servo.begin();
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
    mqttAHT.loop();
    unsigned long now = millis();
    if (now - lastMessage > frequency)
    {
        SensorData data = mqttAHT.readSensor();
        mqttAHT.publishToTopic(data);
        lastMessage = millis();
    }
    if (digitalRead(button_pin) == 0 && now - lastButtonPress > debounceTime)
    {
        if (servo.currentAngle() < 0 || servo.currentAngle() > 50)
        {
            servo.setAngle(0);
        }
        else
        {
            servo.setAngle(120);
        }
        lastButtonPress = millis();
    }
    if (servo.currentAngle() > 50)
    {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
        digitalWrite(BUILTIN_LED, LOW);
    }
    servo.update();
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
