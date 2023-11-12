#ifndef HV513_BANK
#define HV513_BANK

#include <Arduino.h>

class Hv513Bank {
public:
enum class Mode {
    // In source mode, the bank will source current (A 1 bit means HVOUT is High)
    kSource = 0,
    // In Sink mode, the bank will sink current (A 1 bit means HVOUT is Low)
    kSink = 1
  };


  Hv513Bank(uint8_t bank_size, Mode mode, pin_size_t data_pin, pin_size_t clock_pin,
            pin_size_t blank_pin, pin_size_t polarity_pin, pin_size_t latch_enable_pin,
            pin_size_t high_z_pin, pin_size_t short_detect_pin)
    : bank_size_{ bank_size }, mode_{mode}, data_pin_{ data_pin }, clk_pin_{ clock_pin },
      blank_pin_{ blank_pin }, polarity_pin_{ polarity_pin }, latch_enable_pin_{ latch_enable_pin },
      high_z_pin_{ high_z_pin }, short_detect_pin_{ short_detect_pin } {};

  
  void begin();

  // Configure each register in the bank. A 1 in bit j (LSB is 0) of entry i means register i will turn on HVOUT j
  void write(const uint8_t* const bits);

  void blank();
  void unblank();
  // High-Z the outputs
  void enableOutputs();
  // Don't High-Z the outputs
  void disableOutputs();

private:
  uint8_t bank_size_;

  // Whether the register bank is going to source current or sink current. This effectively controls the polarity pin.
  Mode mode_;

  // Pins
  pin_size_t data_pin_;
  pin_size_t clk_pin_;
  pin_size_t blank_pin_;
  pin_size_t polarity_pin_;
  pin_size_t latch_enable_pin_;
  pin_size_t high_z_pin_;
  pin_size_t short_detect_pin_;
};
#endif
