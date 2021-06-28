#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

enum Dir {
  FORWARD = HIGH,
  BACKWARD = LOW
};

class Motor
{
public:
  Motor(int en_pin, int step_pin, int dir_pin, int pulse_w, int step_per_rev);

  void doStep();
  int pos() const;
  Dir dir() const;
  void enable(bool en);
  void setDir(Dir dir);
  void reset(int delay_us);
  void changeDir();
  int stepsPerRevolution() const;

  private:
    const int en_pin_;
    const int step_pin_;
    const int dir_pin_;
    const int pulse_w_;
    const int step_per_rev_;

    int pos_;
    Dir dir_;
};

#endif // MOTOR_H