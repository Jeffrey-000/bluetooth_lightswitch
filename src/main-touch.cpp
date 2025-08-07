#include <Arduino.h>

const char TOUCH_PIN = 32;
const char TOUCH_THRESHOLD = 30;

unsigned long lastTouchTime = 0;
const unsigned long debounceDelay = 500; // milliseconds
bool ledState = false;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  touchAttachInterrupt(TOUCH_PIN, NULL, TOUCH_THRESHOLD);
}

void loop()
{
 int touchValue = touchRead(TOUCH_PIN);

  if (touchValue < TOUCH_THRESHOLD && (millis() - lastTouchTime > debounceDelay)) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    lastTouchTime = millis();
  }
}

