#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), prevError(0), integral(0) {}

float PIDController::update(float target, float current, float dt) {
    float error = target - current;
    integral += error * dt;
    float derivative = (error - prevError) / dt;

    prevError = error;
    return kp * error + ki * integral + kd * derivative;
}

void PIDController::reset() {
    integral = 0;
    prevError = 0;
}
