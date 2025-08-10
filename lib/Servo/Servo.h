#ifndef ESP32_SERVO_PWM_H
#define ESP32_SERVO_PWM_H

#include <Arduino.h>

class Servo
{
public:
    Servo(int gpioPin, int pwmChannel = 0, int freq = 50, int resolution = 16,
          int minUs = 500, int maxUs = 2500, unsigned long sleepDelay = 1000);

    void begin();
    void setAngle(int angle);
    void moveTo(int targetAngle);      // Smoothly move to angle
    void setSpeed(int delayPerStepMs); // Set smooth motion speed (default: 10 ms/step)
    void detach();
    void attach();
    bool isAttached();
    void setUnpoweredIfIdle(bool val, int mosfetPin = -1);
    int currentAngle();
    void update();

private:
    int _pin, _channel, _freq, _resolution, _minUs, _maxUs;
    int _currentAngle = 0;
    int _targetAngle = 0;
    int _speedDelay = 10; // ms delay between steps
    bool _isAttached = false;
    bool _unpoweredIfIdle = false;
    int _mosfetPin = -1;
    void _setAngle(int angle); // Instantly set angle (0–180)
    unsigned long _moveEndTime = 0;
    unsigned long _sleepDelay;
};

#endif
