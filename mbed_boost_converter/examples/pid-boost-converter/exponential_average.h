#pragma once
class ExponentialAverage {
  public:
    ExponentialAverage(float discount_factor) : alpha_{discount_factor} {};

    void AddMeasurement(float measurement){
      memory_ = alpha_ * measurement + (1 - alpha_) * memory_;
    }

    float Output() const {
      return memory_;
    }

    void Clear() { memory_ = 0.0; }

  private:
    float alpha_;
    float memory_ = 0.0;
};
