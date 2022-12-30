#ifndef _OPENTHERM_PID_H_
#define _OPENTHERM_PID_H_

#include <math.h>
#include <stdint.h>

#include "ds.h"

namespace OpenTherm {

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
      cum_err = fmaxf(fminf(cum_err, max), min);
      float delta_input = input - prev_input;

      output = kp * error + cum_err + kd * delta_input;
      output = fmaxf(fminf(output, max), min);

      prev_time = now;
      prev_input = input;
    }

    return output;
  }

  void set_coefficients(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
  }

  void set_setpoint(float sp) { this->setpoint = sp; }

  float get_setpoint() const { return setpoint; }

  float get() const { return output; }

  void set_automatic(bool v, float input) {
    if (v && !automatic)
      initialize(input);
    automatic = v;
  }

protected:
  TimeType time;
  float kp, ki, kd;
  float min, max;
  float output, setpoint, cum_err, prev_input;
  uint64_t prev_time;
  bool automatic;

  void initialize(float input) {
    prev_input = input;
    cum_err = fmaxf(fminf(output, max), min);
    // Note: also recored prev_time = now?
  }
};

} // namespace OpenTherm

#endif // _OPENTHERM_PID_H_