#include "mixer.h"
#include "motor.h"
#include "conf.h"

Mixer::Mixer(const Settings &s, Motor *m)
  : state_(PrepFirstCycle)
  , next_state_(DoNothing)
  , running_(false)
{
  motor_ptr_ = m;

  time_left_us_ = (unsigned long) s.total_time * 1000UL;
  n_repetitions_ = (s.total_time - s.first_cycle_time)
                  / (s.pause_time - s.rep_cycle_time);
  delay_between_steps_us_ = calcStepDelayMicrosec(MIN_ROT_PER_MINUTE,
  					          MAX_ROT_PER_MINUTE,
						  s.speed,
						  m->stepsPerRevolution());

  target_fw_pos_ = (int)((float)s.angle_forward * (m->stepsPerRevolution() / 360.0));
  target_bw_pos_ = -(int)((float)s.angle_backward * (m->stepsPerRevolution() / 360.0));

  settings_ = s;
}

void Mixer::runLoop()
{
  if (!running_) {
    return;
  }

  switch(state_) {
    case PrepFirstCycle:
      current_cycle_left_us_ = (unsigned long)settings_.first_cycle_time * 1000UL;
      prev_millis_ = millis();
      motor_ptr_->setDir(FORWARD);
      next_state_ = PrepPause;
      state_ = MotorRun;
      break;
    case PrepPause:
      current_cycle_left_us_ = (unsigned long)settings_.pause_time * 1000UL;
      next_state_ = PrepRepetition;
      if (motor_ptr_->pos() == 0) {
        state_ = DoNothing;
      } else {
        if (motor_ptr_->pos() < 0) {
          motor_ptr_->setDir(FORWARD);
        } else {
          motor_ptr_->setDir(BACKWARD);
        }
        state_ = MotorReset;
      }
      break;
    case PrepRepetition:
      if (n_repetitions_ == 1) {
        current_cycle_left_us_ = time_left_us_;
        next_state_ = DoNothing;
      } else {
        current_cycle_left_us_ = (unsigned long)settings_.rep_cycle_time * 1000UL;
        next_state_ = PrepPause;
      }
      n_repetitions_--;
      motor_ptr_->setDir(FORWARD);
      state_ = MotorRun;
      break;
    case MotorRun:
      // TODO: After 71 mins there'll be micros() overflow
      if ((micros() - prev_step_time_us_) > delay_between_steps_us_) {
        prev_step_time_us_ = micros();
        motor_ptr_->doStep();
        if ((motor_ptr_->pos() == target_fw_pos_) || (motor_ptr_->pos() == target_bw_pos_)) {
          motor_ptr_->changeDir();
        }
      }
      break;
    case MotorReset:
      // TODO: After 71 mins there'll be micros() overflow
      if ((micros() - prev_step_time_us_) > delay_between_steps_us_) {
        prev_step_time_us_ = micros();
        motor_ptr_->doStep();
        if (motor_ptr_->pos() == 0) {
          state_ = DoNothing;
        }
      }
      break;
    case DoNothing:
      break;
    default:
      break;
  }

  updateTime();
  if (current_cycle_left_us_ == 0) {
    state_ = next_state_;
  }
  if (time_left_us_ == 0) {
    running_ = false;
  }
}

void Mixer::stop()
{
  running_ = false;
  time_left_us_ = 0;
  current_cycle_left_us_ = 0;
}

void Mixer::start()
{
  running_ = true;
}

bool Mixer::isRunning() const
{
  return running_;
}

int Mixer::timeLeft() const
{
  return (int)(time_left_us_ / 1000);
}

int Mixer::timeLeftCycle() const
{
  return (int)(current_cycle_left_us_ / 1000);
}

void Mixer::updateTime()
{
  unsigned long dt = millis() - prev_millis_;
  if (dt) {
    prev_millis_ += dt;
    current_cycle_left_us_ = (current_cycle_left_us_ > dt) ? current_cycle_left_us_ - dt : 0;
    time_left_us_ = (time_left_us_ > dt) ? time_left_us_ - dt : 0;
  }
}

int calcStepDelayMicrosec(int min_rot, int max_rot, int speed_percent, int steps_per_rev)
{
  long rot_per_min = min_rot;
  rot_per_min = (speed_percent - 1);
  rot_per_min *= (max_rot - min_rot);
  rot_per_min = round((rot_per_min) / (100.0 - 1.0));
  rot_per_min += min_rot;

  if (rot_per_min < min_rot) {
    rot_per_min = min_rot;
  } else if (rot_per_min > max_rot) {
    rot_per_min = max_rot;
  }

  // To calc delay in ms between every step according to speed:
  // (60 * 1000 msec/min) / ((rotation/min) * (steps/rotation)) = ms between each step
  unsigned long tmp = 60*1000LU*1000LU;
  tmp /= (rot_per_min * steps_per_rev);
  return (int)tmp;
}