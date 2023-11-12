#include "hv513_bank.h"

void Hv513Bank::begin() {
  pinMode(data_pin_, OUTPUT);
  pinMode(clk_pin_, OUTPUT);
  pinMode(blank_pin_, OUTPUT);
  pinMode(polarity_pin_, OUTPUT);
  pinMode(latch_enable_pin_, OUTPUT);
  pinMode(high_z_pin_, OUTPUT);
  
  // When polarity_{bar} is high, that means a 1 in the data register for output i means HVOUT i is high.
  // This is good for source mode because a 1 bit in the data register should mean we source current.
  //
  // When polarity_{bar} is low, that means a 1 in the data register for output i means HVOUT i is low.
  // This is good for sink mode because a 1 bit in the data register should mean we sink current. 
  auto polarity = (mode_ == Mode::kSource) ? HIGH : LOW;
  digitalWrite(polarity_pin_, polarity);
  digitalWrite(high_z_pin_, HIGH);
}

void Hv513Bank::write(const uint8_t* const bits) {
  // Enable the latch
  digitalWrite(latch_enable_pin_, LOW);
  for (int8_t i = bank_size_ - 1; i >= 0; i--) {
    shiftOut(data_pin_, clk_pin_, MSBFIRST,  bits[i]);
  }
  // Send data from the latch to the outputs
  digitalWrite(latch_enable_pin_, HIGH);
}

void Hv513Bank::blank() {
  digitalWrite(blank_pin_, LOW);
}

void Hv513Bank::unblank() {
  digitalWrite(blank_pin_, HIGH);
}

void Hv513Bank::enableOutputs() {
  digitalWrite(high_z_pin_, HIGH);
}

void Hv513Bank::disableOutputs() {
  digitalWrite(high_z_pin_, LOW);
}
