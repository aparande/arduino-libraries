#include <mbed_boost_converter.h>

#include "mbed.h"
#include "exponential_average.h"

constexpr unsigned int kAdcBits = 12;
constexpr float kAdcRef = 1 << kAdcBits;
constexpr auto kPwmPin = D3;
constexpr auto kVoutPin = A0;
constexpr auto kIndicatorPin = D13;
constexpr float kVref = 3.3;
// Inverse of the voltage divider equation R2 / (R1 + R2)
constexpr float kDividerInv = (1'000'000 + 10'000) / 10'000;

// Operating point
constexpr float kVout = 9;
constexpr float kDuty = 0.5;

// Controller Settings
constexpr auto kControllerPeriodMs = 100;
constexpr auto kMeasurementPeriodUs = 1'000;
constexpr auto kExpAvgAlpha = 0.1;

// Controller boundaries
constexpr float kDMax = 0.95;
constexpr float kDMin = 0.05;
constexpr float kControllerResetVoltage = 2;

// Controller gains
constexpr float k_p = 0.00025;
constexpr float k_d = 0.00025;
constexpr float k_i = 0.001;

ExponentialAverage averager{ kExpAvgAlpha };

constexpr float kPwmPeriodUs = 25;

class BoostConverter : public MbedPidBoostConverter {
public:
  BoostConverter()
    : MbedPidBoostConverter(kPwmPin, kPwmPeriodUs, k_p, k_d, k_i, kVout, kControllerPeriodMs){};

  float readVoltage() final {
    float v_out_counts = analogRead(kVoutPin);
    auto voltage = (v_out_counts / kAdcRef) * kVref;
    auto v_out = voltage * kDividerInv;
    averager.AddMeasurement(v_out);
    return averager.Output();
  }
};

BoostConverter converter;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  converter.begin();
  converter.configureBounds(kDMin, kDMax);

  pinMode(kVoutPin, INPUT);
  analogReadResolution(kAdcBits);

  pinMode(kIndicatorPin, OUTPUT);
}


void Reset() {
  averager.Clear();
  digitalWrite(kIndicatorPin, HIGH);
  converter.reset();
}

void loop() {
  float curr_time = millis();

  if (converter.readVoltage() < kControllerResetVoltage) {
    Reset();
  } else {
    converter.step(curr_time);
    digitalWrite(kIndicatorPin, LOW);
  }

  delayMicroseconds(kMeasurementPeriodUs);
}
