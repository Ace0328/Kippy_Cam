#include "motor.h"

Motor::Motor(int en_pin, int step_pin, int dir_pin, int pulse_w, int step_per_rev_)
  : en_pin_(en_pin)
  , step_pin_(step_pin)
  , dir_pin_(dir_pin)
  , pulse_w_(pulse_w)
  , step_per_rev_(step_per_rev_)
  , pos_(0)
{
  pinMode(en_pin_, OUTPUT);
  pinMode(step_pin_, OUTPUT);
  pinMode(dir_pin_, OUTPUT);

  setDir(FORWARD);
  enable(false);
}

void Motor::doStep()
{
  digitalWrite(step_pin_, HIGH);
  delayMicroseconds(pulse_w_);
  digitalWrite(step_pin_, LOW);

  if (dir_ == FORWARD) {
    pos_++;
  } else {
    pos_--;
  }
}

int Motor::pos() const
{
  return pos_;
}

Dir Motor::dir() const
{
  return dir_;
}

void Motor::enable(bool en)
{
  if (en) {
    digitalWrite(en_pin_, HIGH);
  } else {
    digitalWrite(en_pin_, LOW);
  }
  delayMicroseconds(50); // TODO: The magic. Just delay before TB6600 detect the raise
}

void Motor::setDir(Dir dir)
{
  dir_ = dir;
  digitalWrite(dir_pin_, dir_);
}

void Motor::reset(int delay_us)
{
  if (pos_ > 0) {
    setDir(BACKWARD);
  } else {
    setDir(FORWARD);
  }
  pos_ %= step_per_rev_;
  while (pos_ != 0) {
    doStep();
    delayMicroseconds(delay_us);
  }
  setDir(FORWARD);
}

void Motor::changeDir()
{
  if (dir_ == FORWARD) {
    setDir(BACKWARD);
  } else {
    setDir(FORWARD);
  }
}