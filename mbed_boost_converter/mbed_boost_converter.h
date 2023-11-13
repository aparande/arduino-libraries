#ifndef MBED_BOOST_CONVERTER
#define MBED_BOOST_CONVERTER

#include "mbed.h"
#include <Arduino.h>

class MbedBoostConverter {
public:
  MbedBoostConverter(pin_size_t pwm_pin, uint32_t pwm_period_us,
                     float target_voltage)
      : pwm_pin_{digitalPinToPinName(pwm_pin)},
        pwm_period_us_{pwm_period_us}, vout_{target_voltage} {};

  virtual float readVoltage() = 0;
  virtual void step(float now) = 0;

  void configureBounds(float duty_min, float duty_max);

  virtual void reset();
  void begin();

protected:
  // Pins
  mbed::PwmOut pwm_pin_;

  // Settings
  uint32_t pwm_period_us_;
  float vout_;

  float starting_duty_{0.5};
  float duty_min_{0.01};
  float duty_max_{0.99};
};

class MbedPidBoostConverter : public MbedBoostConverter {
public:
  MbedPidBoostConverter(pin_size_t pwm_pin, uint32_t pwm_period_us, float kp,
                        float kd, float ki, float target_voltage,
                        uint32_t controller_period_ms)
      : MbedBoostConverter(pwm_pin, pwm_period_us, target_voltage),
        controller_period_ms_{controller_period_ms}, kp_{kp}, kd_{kd},
        ki_{ki} {};

  void step(float now_ms) final;
  void reset() final;

private:
  uint32_t controller_period_ms_;

  // Controller gains
  float kp_;
  float kd_;
  float ki_;

  // State
  float last_error_{0};
  bool valid_last_error_{false};
  float integrated_error_{0};
  float next_time_to_act_{0};
};

class MbedStepBoostConverter : public MbedBoostConverter {
public:
  struct Settings {
    float step_size;
    // Converged when target - tolerance <= feedback <= target + tolerance
    float tolerance_v;
    float idle_duty;
  };

  enum class State {
    kIdle = 1,
    kConverging = 2,
    kConverged = 3,
    kDiverged = 4,
  };

  MbedStepBoostConverter(pin_size_t pwm_pin, uint32_t pwm_period_us,
                         float target_voltage, Settings settings)
      : MbedBoostConverter(pwm_pin, pwm_period_us, target_voltage),
        settings_{settings} {
    duty_ = settings_.idle_duty;
  };

  void step(float now_ms) final;
  void reset() final;

  State state() { return state_; };

private:
  float duty_;
  Settings settings_;
  State state_{State::kIdle};
};

#endif
