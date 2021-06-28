#include "buzzer.h"
#include <Arduino.h>

Buzzer::Buzzer(int pin, unsigned int freq_hz, unsigned int interval_ms)
  : pin_(pin)
  , freq_hz_(freq_hz)
  , interval_ms_(interval_ms)
  , prev_millis_(0)
  , running_(false)
  , tone_active_(false)
{
}

void Buzzer::start()
{
  running_ = true;
  prev_millis_ = millis();
  tone_active_ = false;
  tone(pin_, freq_hz_);
}

void Buzzer::stop()
{
  running_ = false;
}

void Buzzer::loop()
{
  if (!running_) {
    return;
  }

  if ((millis() - prev_millis_) > interval_ms_) {
    prev_millis_ = millis();

    if (tone_active_) {
      noTone(pin_);
      tone_active_ = false;
    } else {
      tone(pin_, freq_hz_);
      tone_active_ = true;
    }
  }
}

bool Buzzer::isOn() const
{
  return running_;
}