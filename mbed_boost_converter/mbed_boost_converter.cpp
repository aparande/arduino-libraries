#include "mbed_boost_converter.h"

void MbedBoostConverter::begin() {
  pwm_pin_.period_us(pwm_period_us_); // 40 kHz
  reset();
}

void MbedBoostConverter::reset() { pwm_pin_.write(starting_duty_); }

void MbedBoostConverter::configureBounds(float duty_min, float duty_max) {
  duty_min_ = duty_min;
  duty_max_ = duty_max;
}

void MbedPidBoostConverter::reset() {
  MbedBoostConverter::reset();

  integrated_error_ = 0.0;
  valid_last_error_ = false;
}

void MbedPidBoostConverter::step(float now_ms) {
  auto fdbk = readVoltage();

  if (now_ms < next_time_to_act_) {
    return;
  }

  next_time_to_act_ = now_ms + controller_period_ms_;
  auto error = fdbk - vout_;
  integrated_error_ += error * (controller_period_ms_ / 1'000.0);

  float derivative_error = 0;
  if (valid_last_error_) {
    derivative_error =
        (error - last_error_) / (controller_period_ms_ / 1'000.0);
  }
  last_error_ = error;
  valid_last_error_ = true;

  auto delta_duty_cycle =
      kp_ * error + ki_ * integrated_error_ + kd_ * derivative_error;
  auto duty_cycle = starting_duty_ + delta_duty_cycle;
  if (duty_cycle > duty_max_) {
    duty_cycle = duty_max_;
  } else if (duty_cycle < duty_min_) {
    duty_cycle = duty_min_;
  }
  pwm_pin_.write(duty_cycle);
}

void MbedStepBoostConverter::step(float now_ms) {
  if (state_ == State::kIdle) {
    state_ = State::kConverging;
    // Start with the highest duty cycle allowed to keep the lowest voltage
    duty_ = duty_max_; 
  }

  auto fdbk = readVoltage();
  if (fdbk > vout_ + settings_.tolerance_v) {
    // Voltage too high, decrease it by raising the duty cycle
    duty_ += settings_.step_size;
  } else if (fdbk < vout_ - settings_.tolerance_v) {
    // Voltage too low, increase it by lowering the duty cycle
    duty_ -= settings_.step_size;
  } else {
    state_ = State::kConverged;
  }

  if (duty_ > duty_max_ || duty_ < duty_min_) {
    state_ = State::kDiverged;
  }

  if (state_ == State::kDiverged) {
    pwm_pin_.write(settings_.idle_duty);
  } else {
    pwm_pin_.write(duty_);
  }
}

void MbedStepBoostConverter::reset() {
  state_ = State::kIdle;
  duty_ = settings_.idle_duty;
  pwm_pin_.write(duty_);
}
