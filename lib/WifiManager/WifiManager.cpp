#include <WifiManager.h>

WifiManager::WifiManager(const WifiInfo &wifiInfo, unsigned long MIN_TIMEOUT, unsigned long MAX_TIMEOUT)
    : ssid(wifiInfo.ssid),
      password(wifiInfo.password),
      lastStatus(WL_DISCONNECTED),
      MIN_TIMEOUT(MIN_TIMEOUT),
      MAX_TIMEOUT(MAX_TIMEOUT),
      nextReconnectAttempt(0),
      backoff(MIN_TIMEOUT),
      _client(),

      _timeSyncRequested(false),
      _timeSynced(false),
      _lastTimeSyncMillis(0),
      _lastSyncedEpoch(0),
      _timeSyncInterval(3600000) // 1h default
{
}

// timeoutMs == 0 => non-blocking (don't wait)
// timeoutMs > 0 => block up to that many ms while attempting first connection
void WifiManager::begin(unsigned long timeoutMs)
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());

    if (timeoutMs > 0)
    {
        unsigned long start = millis();
        Serial.print("[WiFi] Connecting");
        while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs)
        {
            delay(500);
            Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED)
        {
            Serial.println("\n[WiFi] Connected");
        }
        else
        {
            Serial.println("\n[WiFi] Connect attempt timed out");
        }
    }
    lastStatus = WiFi.status();
}

void WifiManager::loop()
{
    wl_status_t currentStatus = WiFi.status();

    // If we are disconnected (or not connected)
    if (currentStatus != WL_CONNECTED)
    {
        // First detection of a lost connection (transition WL_CONNECTED -> not connected)
        if (lastStatus == WL_CONNECTED)
        {
            Serial.println("[WiFi] Lost connection!");
            // schedule a reconnect (only if one not already scheduled)
            scheduleReconnect();
        }

        // If a reconnect is scheduled and the time has arrived, do it.
        if (nextReconnectAttempt != 0 && isTimeReached(nextReconnectAttempt))
        {
            reconnect();
        }
    }
    // Regained connection
    else if (currentStatus == WL_CONNECTED && lastStatus != WL_CONNECTED)
    {
        Serial.print("[WiFi] Connected! IP: ");
        Serial.println(WiFi.localIP());
        // Reset backoff and cancel any scheduled attempts
        backoff = MIN_TIMEOUT;
        nextReconnectAttempt = 0;
    }

    lastStatus = currentStatus;

    // Handle NTP sync
    if (_timeSyncRequested)
    {
        _performTimeSync();
    }

    // Periodic resync
    if (periodicTimeSync && _timeSynced && millis() - _lastTimeSyncMillis >= _timeSyncInterval)
    {
        _timeSyncRequested = true;
    }
}

void WifiManager::scheduleReconnect()
{
    // Only schedule if not already scheduled
    if (nextReconnectAttempt == 0)
    {
        Serial.printf("[WiFi] Will try reconnect in %.1f seconds\n", backoff / 1000.0);
        nextReconnectAttempt = millis() + backoff;
        // keep backoff for next time — it will be grown in reconnect() after attempt
    }
}

// Called when the scheduled time has arrived to attempt a reconnect.
// This function initiates a connection and *also* sets the nextReconnectAttempt
// to try again if this attempt doesn't succeed. That ensures repeated attempts.
void WifiManager::reconnect()
{
    Serial.println("[WiFi] Attempting reconnect...");
    WiFi.disconnect(); // clear previous state
    WiFi.begin(ssid.c_str(), password.c_str());

    // schedule next attempt in case this one fails; use current backoff
    // then grow the backoff for the subsequent attempt
    nextReconnectAttempt = millis() + backoff;
    backoff = min(backoff * 2, MAX_TIMEOUT);
}

// safe check for millis() >= target with wrap-around handling
bool WifiManager::isTimeReached(unsigned long target)
{
    // If target == 0 -> not scheduled
    if (target == 0)
        return false;
    // (millis() - target) will be >= 0 when reached, works across wrap
    return (long)(millis() - target) >= 0;
}

wl_status_t WifiManager::getStatus() const
{
    return WiFi.status();
}

wl_status_t WifiManager::getLastStatus() const
{
    return lastStatus;
}

WiFiClient &WifiManager::getClient()
{
    return _client;
}

void WifiManager::syncTime()
{
    _timeSyncRequested = true;
}

void WifiManager::_performTimeSync()
{
    if (getStatus() != WL_CONNECTED)
        return;

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.println("[Time] Waiting for NTP sync...");

    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) // getLocalTime only returns true if valid NTP time is set
    {
        _lastSyncedEpoch = mktime(&timeinfo);
        _lastTimeSyncMillis = millis();
        _timeSynced = true;
        _timeSyncRequested = false;
        Serial.println("[Time] NTP sync complete!");
    }
}

void WifiManager::setPeriodicTimeSync(bool periodic, unsigned long interval_ms)
{
    periodicTimeSync = true;
    _timeSyncInterval = interval_ms;
}