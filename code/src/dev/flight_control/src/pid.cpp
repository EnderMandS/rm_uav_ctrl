#include "pid.h"
#include "ros/time.h"
#include <algorithm>
#include <iostream>

namespace PID {
Pid::Pid(void) {}
Pid::Pid(double p, double i, double d) : p(p), i(i), d(d) { this->reset(); }
Pid::Pid(double p, double i, double d, double p_max, double i_max, double d_max,
         double out_max)
    : p(p), i(i), d(d), p_max(p_max), i_max(i_max), d_max(d_max),
      out_max(out_max) {
  this->reset();
}

void Pid::reset() { expect = out = p_out = i_out = d_out = last_error = 0; }

double Pid::getTime() {
  return ros::Time::now().toSec();
}

void Pid::setPid(double p, double i, double d) {
  this->p = p;
  this->i = i;
  this->d = d;
}

void Pid::setMax(double out_max, double p_max, double i_max, double d_max) {
  this->p_max = p_max;
  this->i_max = i_max;
  this->d_max = d_max;
  this->out_max = out_max;
}

void Pid::setExpect(double expect) { this->expect = expect; }

double Pid::update(double expect, double now) {
  this->expect = expect;
  return update(now);
}

double Pid::update(double now) {
  this->now = now;
  return update();
}

double Pid::update() {
  static double t_last = getTime();
  double t_now = getTime();
  double dt = t_now - t_last;
  t_last = t_now;
  if (dt <= 0) {
    return 0;
  }

  double error = expect - now;
  p_out = dt * p * error;
  i_out += dt * i * error;
  d_out = d * (error - last_error) / dt;
  last_error = error;

  p_out = std::clamp(p_out, -p_max, p_max);
  i_out = std::clamp(i_out, -i_max, i_max);
  d_out = std::clamp(d_out, -d_max, d_max);
  out = p_out + i_out + d_out;
  out = std::clamp(out, -out_max, out_max);
  return out;
}

} // namespace PID