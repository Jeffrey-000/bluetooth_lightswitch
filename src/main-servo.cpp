// #include <Arduino.h>
// const int servoPin = 13;  // GPIO18
// const int pwmChannel = 0;
// const int pwmFreq = 50;   // Standard servo frequency: 50Hz
// const int pwmResolution = 16; // 16-bit resolution

// // Servo pulse width limits in microseconds
// const int minUs = 500;    // ~0 degrees
// const int maxUs = 2500;   // ~180 degrees
// // Maps angle (0–180) to PWM duty
// void setServoAngle(int angle) {
//   angle = constrain(angle, 0, 180);
//   int us = map(angle, 0, 180, minUs, maxUs);
//   uint32_t duty = (uint32_t)((us / 1000000.0) * pwmFreq * (1 << pwmResolution));
//   ledcWrite(pwmChannel, duty);

//   Serial.printf("Angle: %3d -> Duty: %d\n", angle, duty);
// }


// void setup() {
//   Serial.begin(115200);
  
//   // Set up PWM on the specified channel and pin
//   ledcSetup(pwmChannel, pwmFreq, pwmResolution);
//   ledcAttachPin(servoPin, pwmChannel);

//   // Move servo to 0 degrees at startup
//   setServoAngle(0);
// }

// void loop() {
//   // Sweep from 0 to 180 and back
//   for (int angle = 0; angle <= 180; angle += 10) {
//     setServoAngle(angle);
//     delay(500);
//   }
//   for (int angle = 180; angle >= 0; angle -= 10) {
//     setServoAngle(angle);
//     delay(500);
//   }
// }

#include "../lib/Servo/Servo.h"
const int servoPin = 13;  // GPIO18


Servo servo(13);

void setup() {
	servo.begin();
}

void loop() {
  // Sweep from 0 to 180 and back
  for (int angle = 0; angle <= 180; angle += 10) {
    servo.setAngle(angle);
    delay(500);
  }
  for (int angle = 180; angle >= 0; angle -= 10) {
    servo.moveTo(angle);
    delay(500);
  }
}

