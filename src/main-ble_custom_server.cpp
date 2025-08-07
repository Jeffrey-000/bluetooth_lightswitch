#include <Arduino.h>
#include <NimBLEDevice.h>
#include <BLE_CONSTS.h>

#define SERVICE "BAAD"
#define CHR "FOOD"

void initBle();

const char TOUCH_PIN = 32;
const char TOUCH_THRESHOLD = 30;

unsigned long lastTouchTime = 0;
const unsigned long debounceDelay = 500; // milliseconds
bool ledState = false;
bool isConnected = false;

static NimBLEServer *pServer;
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        digitalWrite(BUILTIN_LED, HIGH);
        isConnected = true;
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        isConnected = false;
        digitalWrite(BUILTIN_LED, LOW);
        NimBLEDevice::startAdvertising();
    }
} serverCallbacks;


/** Handler class for characteristic actions */
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
    {
        Serial.printf("%s : onRead(), value: %s\n",
                      pCharacteristic->getUUID().toString().c_str(),
                      pCharacteristic->getValue().c_str());
    }

    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
    {
        Serial.printf("%s : onWrite(), value: %s\n",
                      pCharacteristic->getUUID().toString().c_str(),
                      pCharacteristic->getValue().c_str());
    }

    /**
     *  The value returned in code is the NimBLE host return code.
     */
    void onStatus(NimBLECharacteristic *pCharacteristic, int code) override
    {
        Serial.printf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
    }

    /** Peer subscribed to notifications/indications */
    void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override
    {
        std::string str = "Client ID: ";
        str += connInfo.getConnHandle();
        str += " Address: ";
        str += connInfo.getAddress().toString();
        if (subValue == 0)
        {
            str += " Unsubscribed to ";
        }
        else if (subValue == 1)
        {
            str += " Subscribed to notifications for ";
        }
        else if (subValue == 2)
        {
            str += " Subscribed to indications for ";
        }
        else if (subValue == 3)
        {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID());

        Serial.printf("%s\n", str.c_str());
    }
} chrCallbacks;

void setup(void)
{
    Serial.begin(115200);
    initBle();
    touchAttachInterrupt(TOUCH_PIN, NULL, TOUCH_THRESHOLD);
    pinMode(BUILTIN_LED, OUTPUT);
}

void loop()
{
    /** Loop here and send notifications to connected peers */
    delay(2000);
    int touchValue = touchRead(TOUCH_PIN);

    if (touchValue < TOUCH_THRESHOLD && (millis() - lastTouchTime > debounceDelay))
    {
        if (pServer->getConnectedCount())
        {
            NimBLEService *pSvc = pServer->getServiceByUUID(SERVICE);
            if (pSvc)
            {
                NimBLECharacteristic *pChr = pSvc->getCharacteristic(CHR);
                if (pChr)
                {
                    ledState = !ledState;
                    pChr->setValue(ledState ? "30" : "80");
                    pChr->notify();
                    lastTouchTime = millis();
                }
            }
        }
    }
}

void initBle()
{
    Serial.printf("Starting NimBLE Server\n");

    /** Initialize NimBLE and set the device name */
    NimBLEDevice::init(SERVER_NAME);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCallbacks);

    NimBLEService* lightsService = pServer->createService(SERVICE);
    NimBLECharacteristic* lights_stateCharacteristic =
        lightsService->createCharacteristic(CHR, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    //lights_stateCharacteristic->setValue("off");
    lights_stateCharacteristic->setCallbacks(&chrCallbacks);
    lightsService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName(SERVER_NAME);
    pAdvertising->addServiceUUID(lightsService->getUUID());
    pAdvertising->enableScanResponse(false);
    pAdvertising->start();

    Serial.printf("Advertising Started\n");
}