#include <Arduino.h>

const char PIN = 27;


void setup()
{
    Serial.begin(115200);
  pinMode(PIN, INPUT_PULLUP);
  pinMode(BUILTIN_LED, OUTPUT);
}

void loop()
{
Serial.println(digitalRead(PIN)); //0 == pressed
delay(200);
}

