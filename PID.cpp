#include "PID.h"
#include <Arduino.h>

/**
 * This is copied from: PID Controller Implementation in Software - Phil's Lab #6 
 *
 * See: https://www.youtube.com/watch?v=zOByx3Izf5U
 */

void PID_init(PIDController *pid)
{
  // Clear state
  pid->integrator = 0;
  pid->differentiator = 0;
  pid->prev_error = 0;
  pid->prev_measurement = 0;
}

static void dynamic_integrator_clamping(PIDController* pid, float p)
{
  float limit_max = 0.0, limit_min = 0.0;

  // Dynamic integrator anti-windup
  if (pid->limit_max > p) {
    limit_max = pid->limit_max - p;
  }

  if (pid->limit_min < p) {
    limit_min = pid->limit_min - p;
  }

  if (pid->integrator > limit_max) {
    pid->integrator = limit_max;
  }
  else if (pid->integrator < limit_min) {
    pid->integrator = limit_min;
  }
}

float PID_update(PIDController *pid, float setpoint, float measurement, float dt)
{
  float error = setpoint - measurement;

  // Proportinal term
  float p = pid->Kp * error;

  // Integral term
  pid->integrator += 0.5f * pid->Ki * dt * (error + pid->prev_error);

  // Band limited differentiator
  pid->differentiator = (2.0f * pid->Kd * (measurement - pid->prev_measurement))
                      + (2.0f * pid->tau - dt) * pid->differentiator
                      / (2.0f * pid->tau + dt);

  dynamic_integrator_clamping(pid, p);

  pid->out = p + pid->integrator + pid->differentiator;

  // Clamp output
  if (pid->out > pid->limit_max) {
    pid->out = pid->limit_max;
  }
  else if (pid->out < pid->limit_min) {
    pid->out = pid->limit_min;
  }

  pid->prev_error = error;
  pid->prev_measurement = measurement;

  return pid->out;
}
