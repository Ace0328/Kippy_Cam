#ifndef BUZZER_H
#define BUZZER_H

class Buzzer {

public:
  explicit Buzzer(int pin, unsigned int freq_hz, unsigned int interval_ms);

  void start();
  void stop();
  void loop();
  bool isOn() const;

private:
  const int pin_;
  const unsigned int freq_hz_;
  const unsigned long interval_ms_;

  unsigned long prev_millis_;
  bool running_;
  bool tone_active_;


};
#endif // BUZZER_H