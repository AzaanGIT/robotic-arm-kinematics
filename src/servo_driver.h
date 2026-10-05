/**
 * @file servo_driver.h
 * @brief Smooth motion interpolation and PWM control for multi-joint servos
 */

#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>
#include <Servo.h>

class SmoothServo {
public:
    SmoothServo();
    void attach(int pin, int minPulse = 544, int maxPulse = 2400);
    void setTargetAngle(float target, float speedDegPerSec = 60.0f);
    void update(float dt);
    float getCurrentAngle() const { return currentAngle; }
    bool isMoving() const { return moving; }
    void forceAngle(float angle);

private:
    Servo servo;
    float currentAngle;
    float targetAngle;
    float speed; // deg/sec
    bool moving;
};

#endif // SERVO_DRIVER_H
