#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    float update(float target, float current, float dt);
    void reset();

private:
    float kp, ki, kd;
    float prevError;
    float integral;
};

#endif
