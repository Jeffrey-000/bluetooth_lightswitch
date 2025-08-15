#include "Servo.h"

Servo::Servo(int gpioPin, int pwmChannel, int freq, int resolution,
             int minUs, int maxUs, unsigned long sleepDelay)
    : _pin(gpioPin), _channel(pwmChannel), _freq(freq), _resolution(resolution),
      _minUs(minUs), _maxUs(maxUs), _sleepDelay(sleepDelay) {}

void Servo::begin()
{
    ledcSetup(_channel, _freq, _resolution);
    attach();
    // setAngle(0);
}

void Servo::update()
{
    if (_unpoweredIfIdle && millis() > _moveEndTime && (_currentAngle == _targetAngle))
    {
        if (_mosfetPin > 0)
        {
            digitalWrite(_mosfetPin, LOW); // Turn MOSFET off
        }
        else if (_isAttached)
        {
            detach();
        }
    }
}

void Servo::_setAngle(int angle)
{
    angle = constrain(angle, 0, 180);
    int us = map(angle, 0, 180, _minUs, _maxUs);
    uint32_t duty = (uint32_t)((us / 1000000.0) * _freq * (1 << _resolution));
    ledcWrite(_channel, duty);
    _currentAngle = angle;
}

void Servo::setAngle(int angle)
{
    _targetAngle = constrain(angle, 0, 180);

    if (_unpoweredIfIdle)
    {
        if (_mosfetPin > 0)
        {
            digitalWrite(_mosfetPin, HIGH);
        }
        else if (!_isAttached)
        {
            attach();
        }
    }
    // Estimate movement time (ms) — adjust multiplier for your servo speed
    unsigned long moveTime = abs(_currentAngle - _targetAngle) * _speedDelay; // _speedDelay might be the wrong variable/ will not work with 10?
    _moveEndTime = millis() + moveTime + _sleepDelay;                         // min on for 1 second
    _setAngle(angle);
}

void Servo::moveTo(int targetAngle)
{
    if (_unpoweredIfIdle)
    {
        if (_mosfetPin > 0)
        {
            digitalWrite(_mosfetPin, HIGH);
        }
        else if (!_isAttached)
        {
            attach();
        }
    }
    targetAngle = constrain(targetAngle, 0, 180);
    int step = (targetAngle > _currentAngle) ? 1 : -1;
    for (int angle = _currentAngle; angle != targetAngle; angle += step)
    {
        _setAngle(angle);
        delay(_speedDelay);
    }
    setAngle(targetAngle); // Ensure it finishes exactly
    // performs detach logic if set to true
}

void Servo::setSpeed(int delayPerStepMs)
{
    _speedDelay = max(1, delayPerStepMs); // Avoid 0 delay
}

void Servo::detach()
{
    ledcDetachPin(_pin);
    _isAttached = false;
}

void Servo::attach()
{
    ledcAttachPin(_pin, _channel);
    _isAttached = true;
}

bool Servo::isAttached()
{
    return _isAttached;
}

void Servo::setUnpoweredIfIdle(bool val, int mosfetPin)
{
    _unpoweredIfIdle = val;
    _mosfetPin = mosfetPin;
    if (_mosfetPin > 0)
    {
        pinMode(_mosfetPin, OUTPUT);
        digitalWrite(_mosfetPin, LOW);
    }
}

int Servo::currentAngle()
{
    return _currentAngle;
}