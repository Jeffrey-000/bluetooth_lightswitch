#ifndef ESP32_SERVO_PWM_H
#define ESP32_SERVO_PWM_H

#include <Arduino.h>

class Servo {
public:
    Servo(int gpioPin, int pwmChannel = 0, int freq = 50, int resolution = 16,
                  int minUs = 500, int maxUs = 2500);

    void begin();
    void setAngle(int angle);      // Instantly set angle (0–180)
    void moveTo(int targetAngle);  // Smoothly move to angle
    void setSpeed(int delayPerStepMs); // Set smooth motion speed (default: 10 ms/step)

private:
    int _pin, _channel, _freq, _resolution, _minUs, _maxUs;
    int _currentAngle = 0;
    int _speedDelay = 10; // ms delay between steps
};

#endif
