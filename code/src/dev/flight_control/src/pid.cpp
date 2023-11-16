#include "pid.h"
#include <algorithm>
#include <iostream>

namespace PID {
Pid::Pid(void) {}
Pid::Pid(float p, float i, float d) : p(p), i(i), d(d) { this->reset(); }
Pid::Pid(float p, float i, float d, float p_max, float i_max, float d_max,
         float out_max)
    : p(p), i(i), d(d), p_max(p_max), i_max(i_max), d_max(d_max),
      out_max(out_max) {
  this->reset();
}

void Pid::reset() { expect = out = p_out = i_out = d_out = last_error = 0; }

void Pid::setPid(float p, float i, float d) {
  this->p = p;
  this->i = i;
  this->d = d;
}

void Pid::setMax(float out_max, float p_max, float i_max, float d_max) {
  this->p_max = p_max;
  this->i_max = i_max;
  this->d_max = d_max;
  this->out_max = out_max;
}

void Pid::setExpect(float expect) {
  this->expect = expect;
}

float Pid::update(float t, float now) {
  static float t_last;
  float dt = t-t_last;
  t_last = t;
  if (dt <= 0) {
    std::cout << "PID dt could not less than zero!";
    return 0;
  }

  float error = expect - now;
  p_out = dt * p * error;
  i_out += dt * i * error;
  d_out = dt * d * (error - last_error);
  last_error = error;

  p_out = std::clamp(p_out, -p_max, p_max);
  i_out = std::clamp(i_out, -i_max, i_max);
  d_out = std::clamp(d_out, -d_max, d_max);
  out = p_out + i_out + d_out;
  out = std::clamp(out, -out_max, out_max);
  return out;
}

} // namespace PID