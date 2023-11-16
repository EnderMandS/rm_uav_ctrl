#ifndef __PID_H_
#define __PID_H_

namespace PID {
class Pid {
public:
  Pid(void);
  Pid(float p, float i, float d);
  Pid(float p, float i, float d, float p_max, float i_max, float d_max,
      float out_max);
  float update(float dt, float expect, float now);
  void pidSet(float p, float i, float d);
  void maxSet(float p_max, float i_max, float d_max);
  void reset();

private:
  float p, i, d;
  float out, p_out, i_out, d_out;
  float out_max, p_max, i_max, d_max;
  float last_error;
};
} // namespace PID

#endif // __PID_H_