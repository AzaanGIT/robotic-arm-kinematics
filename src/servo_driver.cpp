#include "servo_driver.h"

SmoothServo::SmoothServo() : currentAngle(90.0f), targetAngle(90.0f), speed(60.0f), moving(false) {}

void SmoothServo::attach(int pin, int minPulse, int maxPulse) {
    servo.attach(pin, minPulse, maxPulse);
    servo.write((int)currentAngle);
}

void SmoothServo::setTargetAngle(float target, float speedDegPerSec) {
    targetAngle = constrain(target, 0.0f, 180.0f);
    speed = speedDegPerSec;
    moving = (fabs(targetAngle - currentAngle) > 0.5f);
}

void SmoothServo::update(float dt) {
    if (!moving) return;

    float diff = targetAngle - currentAngle;
    float maxStep = speed * dt;

    if (fabs(diff) <= maxStep) {
        currentAngle = targetAngle;
        moving = false;
    } else {
        currentAngle += (diff > 0.0f) ? maxStep : -maxStep;
    }

    servo.write((int)round(currentAngle));
}

void SmoothServo::forceAngle(float angle) {
    currentAngle = constrain(angle, 0.0f, 180.0f);
    targetAngle = currentAngle;
    moving = false;
    servo.write((int)round(currentAngle));
}
