#include <mbed_boost_converter.h>

#include "mbed.h"

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
constexpr auto kControllerPeriodMs = 1;

// Controller boundaries
constexpr float kDMax = 0.95;
constexpr float kDMin = 0.05;

constexpr float kPwmPeriodUs = 25;

MbedStepBoostConverter::Settings settings = {.step_size=0.01, .tolerance_v=0.25, .idle_duty=1};

class BoostConverter : public MbedStepBoostConverter {
public:
  BoostConverter()
    : MbedStepBoostConverter(kPwmPin, kPwmPeriodUs, kVout, settings){};

  float readVoltage() final {
    float v_out_counts = analogRead(kVoutPin);
    auto voltage = (v_out_counts / kAdcRef) * kVref;
    return voltage * kDividerInv;
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
  digitalWrite(kIndicatorPin, HIGH);
  converter.reset();
}

void loop() {
  float curr_time = millis();

  if (converter.state() == MbedStepBoostConverter::State::kDiverged) {
    digitalWrite(kIndicatorPin, HIGH);
  } else {
    converter.step(curr_time);
    digitalWrite(kIndicatorPin, LOW);
  }

  delay(kControllerPeriodMs);
}
