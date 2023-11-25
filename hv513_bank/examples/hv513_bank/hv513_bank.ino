#include <hv513_bank.h>

constexpr pin_size_t kBlankingPin = 2;
constexpr pin_size_t kClockPin = 3;
constexpr pin_size_t kDataPin = 4;
constexpr pin_size_t kPolarityPin = 5;
constexpr pin_size_t kLatchPin = 6;
constexpr pin_size_t kHighZPin = 7;
constexpr pin_size_t kHighZCmd = 11;
constexpr pin_size_t kShortDetect = A0;
constexpr pin_size_t kIndicatorPin = 13;

enum Mode {
  kIndividualBlinking,
  kFlash
};

constexpr uint8_t kNumBanks = 5;
// Start only with a particular bank
constexpr uint8_t kBankOffset = 4;
Hv513Bank bank{ kNumBanks, Hv513Bank::Mode::kSink, kDataPin, kClockPin,
                kBlankingPin, kPolarityPin, kLatchPin,
                kHighZPin, kShortDetect };

volatile Mode current_mode = kFlash;

uint8_t value_buf[kNumBanks];
void ChangeMode() {
  if (current_mode == kIndividualBlinking) {
    current_mode = kFlash;
    for (uint8_t i = 0; i < kNumBanks; i++) {
      value_buf[i] = 0xFF;
    }
    bank.write(value_buf);
    Serial.println("Flash");
  } else {
    current_mode = kIndividualBlinking;
    bank.enableOutputs();
    Serial.println("Blink");
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);


  bank.begin();
  pinMode(kHighZCmd, INPUT);
  pinMode(kIndicatorPin, OUTPUT);

  bank.blank();
  bank.enableOutputs();

  attachInterrupt(digitalPinToInterrupt(kHighZCmd), ChangeMode, RISING);
}

uint8_t counter = 0;
void loop() {
  digitalWrite(kIndicatorPin, (current_mode == kIndividualBlinking) ? HIGH : LOW);
  bank.unblank();

  switch (current_mode) {
    case kIndividualBlinking:
      {
        // set everything to 0
        for (uint8_t i = 0; i < kNumBanks; i++) {
          value_buf[i] = 0;
        }
        value_buf[counter / 8] = 1 << (counter % 8);
        bank.write(value_buf);
      }
      break;
    case kFlash:
      if (counter % 2 == 0) {
        bank.enableOutputs();
      } else {
        bank.disableOutputs();
      }
      break;
  }

  delay(1000);
  counter = (counter + 1) % (kNumBanks * 8);
  counter += 8 * kBankOffset * (counter == 0);
}
