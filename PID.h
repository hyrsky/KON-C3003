#ifndef PID_h
#define PID_h

#include <Arduino.h>

struct PIDController {
  // Proportional gain
  float Kp;
  // Integral gain
  float Ki;
  // Derivative gain
  float Kd;

  // Derivative low-pass filter time constant.
  float tau;

  float limit_min;
  float limit_max;

  float integrator;
  float differentiator;
  float prev_error;
  float prev_measurement;

  float out;
};

void PID_init(PIDController *pid);
float PID_update(PIDController *pid, float setpoint, float measurement, float dt);

#endif