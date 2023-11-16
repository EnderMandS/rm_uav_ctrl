#ifndef __PID_H_
#define __PID_H_

namespace PID {
class Pid {
public:
  Pid(void);
  Pid(float p, float i, float d);
  Pid(float p, float i, float d, float p_max, float i_max, float d_max,
      float out_max);
  float update(float t, float now);
  void setPid(float p, float i, float d);
  void setMax(float out_max, float p_max, float i_max, float d_max);
  void setExpect(float expect);
  void reset();
  float expect=0;
  float p=0, i=0, d=0;
  float p_out=0, i_out=0, d_out=0;
  float out_max=9999, p_max=9999, i_max=9999, d_max=9999;

private:
  float out=0, last_error=0;
};
} // namespace PID

#endif // __PID_H_