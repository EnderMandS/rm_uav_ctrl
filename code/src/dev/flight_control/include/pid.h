#ifndef __PID_H_
#define __PID_H_

namespace PID {
class Pid {
public:
  Pid(void);
  Pid(double p, double i, double d);
  Pid(double p, double i, double d, double p_max, double i_max, double d_max,
      double out_max);

  double expect = 0, now = 0;
  double p = 0, i = 0, d = 0;
  double out = 0, p_out = 0, i_out = 0, d_out = 0;
  double out_max = 9999, p_max = 9999, i_max = 9999, d_max = 9999;

  double update(double expect, double now);
  double update(double now);
  double update();
  void setPid(double p, double i, double d);
  void setMax(double out_max, double p_max, double i_max, double d_max);
  void setExpect(double expect);
  void reset();
  double getTime();

private:
  double last_error = 0;
};
} // namespace PID

#endif // __PID_H_