#pragma once
#include <WiFi.h>
struct WifiInfo
{
    std::string ssid;
    std::string password;
};

class WifiManager
{
public:
    WifiManager(const WifiInfo &wifiInfo, unsigned long MIN_TIMEOUT = 5000, unsigned long MAX_TIMEOUT = 300000);
    // default min 5 sec, max 5 min
    void begin(unsigned long timeoutMs = 0);
    void loop();
    wl_status_t getStatus() const;
    wl_status_t getLastStatus() const;
    // Return a reference to an internal WiFiClient you can use for sockets.
    // Returns a non-owning reference (the client is owned by WifiManager).
    WiFiClient &getClient();

    void syncTime(); // Request a sync (called automatically after Wi-Fi)
    void setPeriodicTimeSync(bool periodic, unsigned long interval_ms = 86400000);

private:
    std::string ssid;
    std::string password;
    wl_status_t lastStatus;
    unsigned long MIN_TIMEOUT;
    unsigned long MAX_TIMEOUT;
    unsigned long nextReconnectAttempt;
    unsigned long backoff;
    WiFiClient _client; // <-- single shared WiFiClient instance
    bool isTimeReached(unsigned long target);
    void scheduleReconnect();
    void reconnect();
    // Time sync
    bool _timeSyncRequested;
    bool _timeSynced;
    unsigned long _lastTimeSyncMillis;
    time_t _lastSyncedEpoch;
    unsigned long _timeSyncInterval;

    static bool timeReached(unsigned long target);
    bool periodicTimeSync = false;
    void _performTimeSync();
};
