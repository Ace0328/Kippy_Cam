#ifndef MIXER_H
#define MIXER_H

#define TIME_MAX (3599) // seconds

/* Configurable */
int calcStepDelayMicrosec(int min_rot, int max_rot, int speed_percent, int steps_per_rev);

struct Settings
{
  int total_time;
  int first_cycle_time;
  int pause_time;
  int rep_cycle_time;
  int speed;
  int angle_forward;
  int angle_backward;
};

class Motor;

class Mixer
{
public:
  Mixer(const Settings &s, Motor *m);
  void runLoop();
  void stop();
  void start();
  bool isRunning() const;
  int timeLeft() const;
  int timeLeftCycle() const;

private:
    void updateTime();

private:
    enum State {
      PrepFirstCycle,
      PrepPause,
      PrepRepetition,
      MotorRun,
      MotorReset,
      DoNothing
    };

    Motor *motor_ptr_;

    State state_;
    State next_state_;
    Settings settings_;
    unsigned long time_left_us_;
    unsigned long current_cycle_left_us_;
    unsigned long prev_step_time_us_;
    unsigned long prev_millis_;
    int n_repetitions_;
    unsigned long delay_between_steps_us_;
    int target_fw_pos_;
    int target_bw_pos_;
    bool running_;
};

#endif // MIXER_H