#include "Servo.h"

Servo::Servo(int gpioPin, int pwmChannel, int freq, int resolution,
                             int minUs, int maxUs)
    : _pin(gpioPin), _channel(pwmChannel), _freq(freq), _resolution(resolution),
      _minUs(minUs), _maxUs(maxUs) {}

void Servo::begin() {
    ledcSetup(_channel, _freq, _resolution);
    ledcAttachPin(_pin, _channel);
    setAngle(0);
}

void Servo::setAngle(int angle) {
    angle = constrain(angle, 0, 180);
    int us = map(angle, 0, 180, _minUs, _maxUs);
    uint32_t duty = (uint32_t)((us / 1000000.0) * _freq * (1 << _resolution));
    ledcWrite(_channel, duty);
    _currentAngle = angle;
}

void Servo::moveTo(int targetAngle) {
    targetAngle = constrain(targetAngle, 0, 180);
    int step = (targetAngle > _currentAngle) ? 1 : -1;
    for (int angle = _currentAngle; angle != targetAngle; angle += step) {
        setAngle(angle);
        delay(_speedDelay);
    }
    setAngle(targetAngle); // Ensure it finishes exactly
}

void Servo::setSpeed(int delayPerStepMs) {
    _speedDelay = max(1, delayPerStepMs); // Avoid 0 delay
}
