#ifndef _OPENTHERM_PID_H_
#define _OPENTHERM_PID_H_

#include <stdint.h>

// From Brett Beauregard's excellent series of blog posts:
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
template <typename TimeType> class PIDController {
public:
  PIDController(float kp = 0.0, float ki = 0.0, float kd = 0.0, float min = 0.0,
                float max = 0.0)
      : kp(kp), ki(ki), kd(kd), min(min), max(max), output(0.0), setpoint(0.0),
        cum_err(0.0), prev_input(0.0), automatic(false) {}

  virtual ~PIDController() = default;

  float update(float input) {
    if (automatic) {
      uint64_t now = time.get_us();
      float delta_t = (float)(now - prev_time);

      float error = setpoint - input;
      cum_err += ki * (error * delta_t);
      if (cum_err > max)
        cum_err = max;
      else if (cum_err < min)
        cum_err = min;
      float delta_input = input - prev_input;

      output = kp * error + cum_err + kd * delta_input;
      if (output > max)
        output = max;
      else if (output < min)
        output = min;

      prev_time = now;
      prev_input = input;
    }

    return output;
  }

  void set(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
  }

  float get() const { return output; }

  void set_automatic(bool v, float input) {
    if (v && !automatic)
      initialize(input);
    automatic = v;
  }

  void initialize(float input) {
    prev_input = input;
    cum_err = output;
    if (cum_err > max)
      cum_err = max;
    else if (cum_err < min)
      cum_err = min;
  }

protected:
  TimeType time;
  float kp, ki, kd;
  float min, max;
  float output, setpoint, cum_err, prev_input;
  uint64_t prev_time;
  bool automatic;
};

#endif // _OPENTHERM_PID_H_