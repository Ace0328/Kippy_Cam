#include <math.h>
#include <Nextion.h>

/* Used Pins */

#define dirPin  12
#define stepPin 13

enum Dir {
  FORWARD = HIGH,
  BACKWARD = LOW
};

/* Macro-defined constants */

#define MIN_ROT_PER_MINUTE 1
#define MAX_ROT_PER_MINUTE 50

/*TB66000 configuration*/
#define TB6600_STEPS_PER_REV  3200
#define TB6600_PULSE_WIDTH_US 5

#define TIM_MS_INT_PRINT    (200U)
#define TIM_MS_INT_GUI      (30U) // To read Nex and check holding
#define TIM_MS_INT_CONTROL  (1U)

#define TIME_MAX (3599) // seconds

#define PRESET_TOTAL_TIME       (10 * 60) // 10 minutes = 600 seconds
#define PRESET_FIRST_CYCLE_TIME (30)      // 30 seconds
#define PRESET_PAUSE_TIME       (50)      // 50 seconds
#define PRESET_REP_CYCLE_TIME   (10)      // 10 seconds

#define PRESET_ANGLE_FW (185)
#define PRESET_ANGLE_BW (80)
#define PRESET_SPEED    (85)

/* Variable constants  */
const int increment=1;
const int time_click=5; //wait 5ms
const int max_speed=3;  //delay(1ms faster speed)
const int min_speed=15; //delay(500ms slower speed)

/* Function prototypes */
void StartPushCallback(void *ptr);
void StopPushCallback(void *ptr);
void ResetPushCallback(void *ptr);
void LeftPushCallback(void *ptr);
void LeftPopCallback(void *ptr);
void RightPushCallback(void *ptr);
void RightPopCallback(void *ptr);
void UP1PushCallback(void *ptr);
void UP1PopCallback(void *ptr);
void DW1PushCallback(void *ptr);
void DW1PopCallback(void *ptr);
void UP2PushCallback(void *ptr);
void UP2PopCallback(void *ptr);
void DW2PushCallback(void *ptr);
void DW2PopCallback(void *ptr);
void UP3PushCallback(void *ptr);
void UP3PopCallback(void *ptr);
void DW3PushCallback(void *ptr);
void DW3PopCallback(void *ptr);
void UP4PushCallback(void *ptr);
void UP4PopCallback(void *ptr);
void DW4PushCallback(void *ptr);
void DW4PopCallback(void *ptr);
void UP5PushCallback(void *ptr);
void UP5PopCallback(void *ptr);
void DW5PushCallback(void *ptr);
void DW5PopCallback(void *ptr);
void UP6PushCallback(void *ptr);
void UP6PopCallback(void *ptr);
void DW6PushCallback(void *ptr);
void DW6PopCallback(void *ptr);
void UP7PushCallback(void *ptr);
void UP7PopCallback(void *ptr);
void DW7PushCallback(void *ptr);
void DW7PopCallback(void *ptr);

void resetSettingsToDefault();
void updateNexVal(const char *name, unsigned int val);
void updateTime(const char *min_name, const char *sec_name, int time_sec);
void displayTimeLeft();
void displaySettings();
void handleHoldButtons();
void doMotorStep();
void motorReset();
int calcStepDelayMicrosec(int speed_percent);
void runControl_not_blocking(unsigned long time_ms);
void runControl();
void motorTestControl();

class MotorNew
{
public:
  MotorNew(int step_pin, int dir_pin, int pulse_w)
    : step_pin_(step_pin)
    , dir_pin_(dir_pin)
    , pulse_w_(pulse_w)
    , pos_(0)
  {
    pinMode(step_pin_, OUTPUT);
    pinMode(step_pin_, OUTPUT);

    setDir(FORWARD);
  }

  void doStep() {
    digitalWrite(step_pin_, HIGH);
    delayMicroseconds(pulse_w_);
    digitalWrite(step_pin_, LOW);

    if (dir_ == FORWARD) {
      pos_++;
    } else {
      pos_--;
    }
  }

  int pos() const {
    return pos_;
  }

  Dir dir() const {
    return dir_;
  }

  void setDir(Dir dir) {
    dir_ = dir;
    digitalWrite(dir_pin_, dir_);
  }

  void reset(int delay_us) {
    if (pos_ > 0) {
      setDir(BACKWARD);
    } else {
      setDir(FORWARD);
    }
    pos_ %= TB6600_STEPS_PER_REV;
    Serial.println(__func__);
    Serial.println(pos_);
    while (pos_ != 0) {
      doStep();
      delayMicroseconds(delay_us);
    }
    setDir(FORWARD);
  }

  void changeDir() {
    Serial.println(__func__);
    if (dir_ == FORWARD) {
      setDir(BACKWARD);
    } else {
      setDir(FORWARD);
    }
  }

  private:
    const int step_pin_;
    const int dir_pin_;
    const int pulse_w_;

    int pos_;
    Dir dir_;
};

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

class Mixer
{
  public:
  Mixer(const Settings &s, MotorNew *m)
    : state_(PrepFirstCycle)
    , next_state_(DoNothing)
  {
    motor_ptr_ = m;

    time_left_us_ = (unsigned long) s.total_time * 1000UL;
    n_repetitions_ = (s.total_time - s.first_cycle_time)
                    / (s.pause_time - s.rep_cycle_time);
    delay_between_steps_us_ = calcStepDelayMicrosec(s.speed);

    target_fw_pos_ = (int)((float)s.angle_forward * (TB6600_STEPS_PER_REV / 360.0));
    target_bw_pos_ = -(int)((float)s.angle_backward * (TB6600_STEPS_PER_REV / 360.0));

    Serial.print("Time left us = "); Serial.println(time_left_us_);
    Serial.print("N rep = "); Serial.println(n_repetitions_);
    Serial.print("Delay step = "); Serial.println(delay_between_steps_us_);
    Serial.print("Target FW pos = "); Serial.println(target_fw_pos_);
    Serial.print("Target BW pos = "); Serial.println(target_bw_pos_);

    settings_ = s;
  }

  void runLoop() {

    if (!running_) {
      return;
    }

    switch(state_) {
      case PrepFirstCycle:
        Serial.println("FirstCycle started");
        current_cycle_left_us_ = (unsigned long)settings_.first_cycle_time * 1000UL;
        prev_millis_ = millis();
        motor_ptr_->setDir(FORWARD);
        next_state_ = PrepPause;
        state_ = MotorRun;
        break;
      case PrepPause:
        Serial.println("Pause started");
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
        Serial.println("Repetition started");
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
      Serial.println("Time left");
      running_ = false;
    }
  }

  void Stop() {
    running_ = false;
  }

  void Start() {
    running_ = true;
  }

  bool isRunning() const {
    return running_;
  }

  int timeLeft() {
    return (int)(time_left_us_ / 1000);
  }

  int timeLeftCycle() {
    return (int)(current_cycle_left_us_ / 1000);
  }

  private:
    void updateTime() {
      unsigned long dt = millis() - prev_millis_;
      if (dt) {
        prev_millis_ += dt;
        current_cycle_left_us_ = (current_cycle_left_us_ > dt) ? current_cycle_left_us_ - dt : 0;
        time_left_us_ = (time_left_us_ > dt) ? time_left_us_ - dt : 0;
      }
    }

  private:
    enum State {
      PrepFirstCycle,
      PrepPause,
      PrepRepetition,
      MotorRun,
      MotorReset,
      DoNothing
    };

    MotorNew *motor_ptr_;

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

/* private variable */
MotorNew motor_new = MotorNew(stepPin, dirPin, TB6600_PULSE_WIDTH_US);
Mixer mixer = Mixer(Settings(), &motor_new);

int total_time_;
int first_cycle_time_;
int pause_time_;
int rep_cycle_time_;
int current_cycle_time_;
unsigned long time_left_ms_;
unsigned long current_cycle_left_ms_;

struct Motor {
  int pos;
  int target_fw_pos;
  int target_bw_pos;
  int current_dir;
};
Motor motor = {0, 0, 0, FORWARD};

bool running_;

unsigned int A_AF;
unsigned int A_AR;
int A_Speed;

int i=0;
int pos=0;
unsigned int Step_F=0;
unsigned int Step_R=0;
unsigned int r=0;

/* Counters to call function with some periodic interval */
unsigned long prev_millis_for_GUI = 0;
unsigned long prev_millis_for_print = 0;
unsigned long prev_millis_for_control = 0;

/* Depricated */
unsigned int delay_motion=0;//delay for 1 step ms

unsigned int A_ST = 0;
unsigned int A_CT = 0;
unsigned int A_F_cycle = 0;
unsigned int A_P = 0;
unsigned int A_R = 0;

/* Nextion buttons and texts */
NexButton Start = NexButton(0, 15, "Start");  // Button added
NexButton Stop = NexButton(0, 16, "Stop");  // Button added
NexButton Reset = NexButton(0, 17, "Reset");  // Button added
NexButton Left = NexButton(0, 18, "Left");  // Button added
NexButton Right = NexButton(0, 19, "Right");  // Button added

NexButton UP1 = NexButton(0, 4, "UP1");  // Button added
NexButton UP2 = NexButton(0, 7, "UP2");  // Button added
NexButton UP3 = NexButton(0, 10, "UP3");  // Button added
NexButton UP4 = NexButton(0, 13, "UP4");  // Button added
NexButton UP5 = NexButton(0, 21, "UP5");  // Button added
NexButton UP6 = NexButton(0, 24, "UP6");  // Button added
NexButton UP7 = NexButton(0, 47, "UP7");  // Button added

NexButton DW1 = NexButton(0, 5, "DW1");  // Button added
NexButton DW2 = NexButton(0, 8, "DW2");  // Button added
NexButton DW3 = NexButton(0, 11, "DW3");  // Button added
NexButton DW4 = NexButton(0, 14, "DW4");  // Button added
NexButton DW5 = NexButton(0, 22, "DW5");  // Button added
NexButton DW6 = NexButton(0, 25, "DW6");  // Button added
NexButton DW7 = NexButton(0, 48, "DW7");  // Button added

NexNumber ST_m = NexNumber(0, 34, "ST_m");  // Number added
NexNumber ST_s = NexNumber(0, 35, "ST_s");  // Number added

NexNumber F_cycle_m = NexNumber(0, 36, "F_cycle_m");  // Number added
NexNumber F_cycle_s = NexNumber(0, 38, "F_cycle_s");  // Number added

NexNumber P_m = NexNumber(0, 39, "P_m");  // Number added
NexNumber P_s = NexNumber(0, 41, "P_s");  // Number added

NexNumber R_m = NexNumber(0, 42, "R_m");  // Number added  Rep. cycle(munite)
NexNumber R_s = NexNumber(0, 44, "R_s");  // Number added  Rep. cycle(second)

NexNumber AF = NexNumber(0, 45, "AF");  // Number added  Angle FWD
NexNumber AR = NexNumber(0, 46, "AR");  // Number added  Angle REVERS

NexNumber Speed = NexNumber(0, 49, "Speed");  // Number added  Angle REVERS

// Button hold flags
bool up1=false;
bool up2=false;
bool up3=false;
bool up4=false;
bool up5=false;
bool up6=false;
bool up7=false;
bool dw1=false;
bool dw2=false;
bool dw3=false;
bool dw4=false;
bool dw5=false;
bool dw6=false;
bool dw7=false;

bool start_shaking_ = false;
bool stop_shaking_ = false;
bool A_Left = false;
bool A_Right = false;

/* List of touchable elements */
NexTouch *nex_listen_list[] =
{
  &Start,  // Button added
  &Stop,  // Button added
  &Reset,  // Button added
  &Left,  // Button added
  &Right,  // Button added
  &UP1,  // Button added
  &UP2,  // Button added
  &UP3,  // Button added
  &UP4,  // Button added
  &UP5,  // Button added
  &UP6,  // Button added
  &UP7,  // Button added
  &DW1,  // Button added
  &DW2,  // Button added
  &DW3,  // Button added
  &DW4,  // Button added
  &DW5,  // Button added
  &DW6,  // Button added
  &DW7,  // Button added
  NULL  // String terminated
};  // End of touch event list


/*====================================================================*/
void setup() {

  Serial.begin(9600); // Start serial comunication at baud=9600
  // delay(500);  // This dalay is just in case the nextion display didn't start yet, to be sure it will receive the following command.
  //  Serial.print("baud=115200");  // Set new baud rate of nextion to 115200, but it's temporal. Next time nextion is power on,
  //  Serial.write(0xff);  // We always have to send this three lines after each command sent to nextion.
  //  Serial.write(0xff);
  //  Serial.write(0xff);
  //  Serial.end();  // End the serial comunication of baud=9600
  // Serial.begin(115200);  // Start serial comunication at baud=115200

  nexInit();

  // Register the event callback functions of each touch event:
  Start.attachPush(StartPushCallback, &Start); // Button press
  Stop.attachPush(StopPushCallback, &Stop);    // Button press
  Reset.attachPush(ResetPushCallback, &Reset); // Button press
  Left.attachPush(LeftPushCallback, &Left);    // Button press
  Left.attachPop(LeftPopCallback, &Left);      // Button press
  Right.attachPush(RightPushCallback, &Right); // Button press
  Right.attachPop(RightPopCallback, &Right);   // Button press

  UP1.attachPush(UP1PushCallback, &UP1); // Button press
  UP2.attachPush(UP2PushCallback, &UP2); // Button press
  UP3.attachPush(UP3PushCallback, &UP3); // Button press
  UP4.attachPush(UP4PushCallback, &UP4); // Button press
  UP5.attachPush(UP5PushCallback, &UP5); // Button press
  UP6.attachPush(UP6PushCallback, &UP6); // Button press
  UP7.attachPush(UP7PushCallback, &UP7); // Button press
  UP1.attachPop(UP1PopCallback, &UP1);   // Button press
  UP2.attachPop(UP2PopCallback, &UP2);   // Button press
  UP3.attachPop(UP3PopCallback, &UP3);   // Button press
  UP4.attachPop(UP4PopCallback, &UP4);   // Button press
  UP5.attachPop(UP5PopCallback, &UP5);   // Button press
  UP6.attachPop(UP6PopCallback, &UP6);   // Button press
  UP7.attachPop(UP7PopCallback, &UP7);   // Button press

  DW1.attachPush(DW1PushCallback, &DW1); // Button press
  DW2.attachPush(DW2PushCallback, &DW2); // Button press
  DW3.attachPush(DW3PushCallback, &DW3); // Button press
  DW4.attachPush(DW4PushCallback, &DW4); // Button press
  DW5.attachPush(DW5PushCallback, &DW5); // Button press
  DW6.attachPush(DW6PushCallback, &DW6); // Button press
  DW7.attachPush(DW7PushCallback, &DW7); // Button press
  DW1.attachPop(DW1PopCallback, &DW1);   // Button press
  DW2.attachPop(DW2PopCallback, &DW2);   // Button press
  DW3.attachPop(DW3PopCallback, &DW3);   // Button press
  DW4.attachPop(DW4PopCallback, &DW4);   // Button press
  DW5.attachPop(DW5PopCallback, &DW5);   // Button press
  DW6.attachPop(DW6PopCallback, &DW6);   // Button press
  DW7.attachPop(DW7PopCallback, &DW7);   // Button press

  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  resetSettingsToDefault();

  start_shaking_ = false;
  stop_shaking_ = false;
  running_ = true;

  Settings tmp = {.total_time = 120,
                  .first_cycle_time = 30,
                  .pause_time = 30,
                  .rep_cycle_time = 10,
                  .speed = 85,
                  .angle_forward = 270,
                  .angle_backward = 270};


  mixer = Mixer(tmp, &motor_new);

  //for (int i = 0; i < 200; i++) {
    //motor_new.doStep();
    //delay(3);
  //}
  mixer.Start();
}

void loop()
{
  if ((millis() % 500) == 0) {
    delay(10);
  }

  if (mixer.isRunning()) {
    mixer.runLoop();

    // 10 sec off
    if (millis() > 10000) {
      Serial.println("Timeout stop");
      mixer.Stop();
      motor_new.reset(375);
    }
  }

  // const static int step_delay = calcStepDelayMicrosec(50);
  // static unsigned long prev_step_time_us = 0;

  // if ((millis() % 500) == 0) {
  //   delay(10);
  // }

  // if (running_) {
  //   if ((micros() - prev_step_time_us) > step_delay) {
  //     prev_step_time_us = micros();
  //     motor_new.doStep();

  //     if ((motor_new.pos() == 800) || (motor_new.pos() == -800)) {
  //       motor_new.changeDir();
  //       Serial.println("Change dir");
  //     }
  //   }
  //   if (millis() > 10000) {
  //     running_ = false;
  //     Serial.println("Reseting");
  //     motor_new.reset(step_delay);
  //   }
  // }


  // Read GUI and handle buttons every TIM_MS_INT_GUI (1000/TIM_MS_INT_GUI per sec)
  // if ((millis() - prev_millis_for_GUI) >= TIM_MS_INT_GUI) {
  //   prev_millis_for_GUI = millis();
  //   // It's blocking (while serial.available)
  //   // and use delay for 10 ms;
  //   // Though, if Serial.Available == 0, that it just skips
  //   nexLoop(nex_listen_list); // Check for any touch event
  //   if (!running_) {
  //     handleHoldButtons();
  //   }
  // }

  // Update values evert TIM_MS_INT_PRINT ms
  // if ((millis() - prev_millis_for_print) >= TIM_MS_INT_PRINT) {
  //   prev_millis_for_print = millis();
  //   if (!running_) {
  //     displaySettings();
  //   } else {
  //     displayTimeLeft();
  //   }
  // }

  // if (start_shaking_) {
  //   // TODO: Verify all time settings before running
  //   running_ = true;
  //   start_shaking_ = false;

  // }

  // if (running_) {
  //   // Call control function every TIM_MS_INT_CONTROL
  //   // It also handles if more than 1 ms passed (in case of print)
  //   if ((millis() - prev_millis_for_control) >= TIM_MS_INT_CONTROL) {
  //     prev_millis_for_control = millis();
  //     runControl_not_blocking(prev_millis_for_control);
  //   }

  //   if (stop_shaking_) {
  //     running_ = false;
  //     stop_shaking_ = false;
  //     current_cycle_left_ms_ = 0;
  //     time_left_ms_ = 0;
  //     motorReset();
  //     displayTimeLeft();
  //   }
  // } else {
  //   // TODO: Should be refactored
  //   // Turn left if the button is held
  //   if (A_Left) {
  //     digitalWrite(dirPin, LOW);
  //   }
  //   while (A_Left) {
  //     digitalWrite(stepPin, HIGH);
  //     delay(delay_motion * 10);
  //     nexLoop(nex_listen_list); // Check for any touch event

  //     digitalWrite(stepPin, LOW);
  //     delay(delay_motion * 10);
  //     nexLoop(nex_listen_list); // Check for any touch event
  //   }

  //   // Turn right if the button is held
  //   if (A_Right) {
  //     digitalWrite(dirPin, HIGH);
  //   }
  //   while (A_Right) {
  //     digitalWrite(stepPin, HIGH);
  //     delay(delay_motion * 10);
  //     nexLoop(nex_listen_list); // Check for any touch event

  //     digitalWrite(stepPin, LOW);
  //     delay(delay_motion * 10);
  //     nexLoop(nex_listen_list); // Check for any touch event
  //   }
  // }
}

void resetSettingsToDefault()
{
  total_time_ = PRESET_TOTAL_TIME;
  first_cycle_time_ = PRESET_FIRST_CYCLE_TIME;
  pause_time_ = PRESET_PAUSE_TIME;
  pause_time_ = PRESET_REP_CYCLE_TIME;

  A_AF = PRESET_ANGLE_FW;
  A_AR = PRESET_ANGLE_BW;
  A_Speed = PRESET_SPEED;
}

//---------Start button-------------------------------------------------
void StartPushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  start_shaking_ = true;
}

//---------Stop button-------------------------------------------------
void StopPushCallback(void *ptr)
{
  stop_shaking_ = true;
}

//---------Reset button-------------------------------------------------
void ResetPushCallback(void *ptr)
{
  if (running_) {
    return;
}
  resetSettingsToDefault();
  motorReset();
}

//---------Left button-------------------------------------------------
void LeftPushCallback(void *ptr)
{
  A_Left = true;
  A_Right = false;
}

void LeftPopCallback(void *ptr)
{
  A_Left = false;
}

//---------Right button-------------------------------------------------
void RightPushCallback(void *ptr)
{
  A_Right = true;
  A_Left = false;
}

void RightPopCallback(void *ptr)
{
  A_Right = false;
}

//---------Total time-------------------------------------------------
void UP1PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  total_time_ = min(total_time_ + increment, TIME_MAX);
  delay(time_click);
  i=0;
  up1=true;
}

void UP1PopCallback(void *ptr)
{
  up1=false;
}

void DW1PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  total_time_ = max(total_time_ - increment, 0);
  delay(time_click);
  i=0;
  dw1=true;
}

void DW1PopCallback(void *ptr)
{
  dw1=false;
}

//---------First cycle-------------------------------------------------
void UP2PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  // TODO: check if total time became more that TIME_MAX
  first_cycle_time_ = min(first_cycle_time_ + increment,  TIME_MAX);
  delay(time_click);
  i=0;
  up2=true;
}

void UP2PopCallback(void *ptr)
{
  up2=false;
}

void DW2PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  first_cycle_time_ = max(first_cycle_time_ - increment, 0);
  delay(time_click);
  i=0;
  dw2=true;
}

void DW2PopCallback(void *ptr)
{
  dw2=false;
}

//---------Pause time-------------------------------------------------
void UP3PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  // TODO: check if total time became more that TIME_MAX
  pause_time_ = min(pause_time_ + increment, TIME_MAX);
  delay(time_click);
  i=0;
  up3=true;
}

void UP3PopCallback(void *ptr)
{
  up3=false;
}

void DW3PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  pause_time_ = max(pause_time_ - increment, 0);
  delay(time_click);
  i=0;
  dw3=true;
}

void DW3PopCallback(void *ptr)
{
  dw3=false;
}

//---------Repetition cycle-------------------------------------------------
void UP4PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  // TODO: check if total time became more that TIME_MAX
  rep_cycle_time_ = min(rep_cycle_time_ + increment, TIME_MAX);
  delay(time_click);
  i=0;
  up4=true;
}

void UP4PopCallback(void *ptr)
{
  up4=false;
}

void DW4PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  rep_cycle_time_ = max(rep_cycle_time_ - increment, 0);
  delay(time_click);
  i=0;
  dw4=true;
}
void DW4PopCallback(void *ptr)
{
  dw4=false;
}

//---------Angle FORWARD-------------------------------------------------
void UP5PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  A_AF = A_AF + increment;
  delay(time_click);
  i=0;
  up5=true;
}

void UP5PopCallback(void *ptr)
{
  up5=false;
}

void DW5PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  A_AF = A_AF - increment;
  if(A_AF<=0)A_AF=0;
  delay(time_click);
  i=0;
  dw5=true;
}

void DW5PopCallback(void *ptr)
{
  dw5=false;
}

//---------Angle REVERS-------------------------------------------------
void UP6PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  A_AR = A_AR + increment;
  delay(time_click);
  i=0;
  up6=true;
}

void UP6PopCallback(void *ptr)
{
  up6=false;
}

void DW6PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  A_AR = A_AR - increment;
  if(A_AR<=0)A_AR=0;
  delay(time_click);
  i=0;
  dw6=true;
}

void DW6PopCallback(void *ptr)
{
  dw6=false;
}

//---------Speed-------------------------------------------------
void UP7PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  A_Speed = A_Speed + increment;
  if(A_Speed>=100)
  A_Speed=100;
  Serial.println("");
  Serial.println("========================");
  up7=true;
  delay(time_click);
  i=0;
}

void UP7PopCallback(void *ptr)
{
  up7=false;
}

void DW7PushCallback(void *ptr)
{
  if (running_) {
    return;
  }
  A_Speed = A_Speed - increment;
  if(A_Speed<=0)
  A_Speed=0;
  delay(time_click);
  i=0;
  dw7=true;
}

void DW7PopCallback(void *ptr)
{
  dw7=false;
}

void updateNexVal(const char *name, unsigned int val)
{
  Serial.print(name);
  Serial.print(val);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

void updateTime(const char *min_name, const char *sec_name, int time_sec)
{
  updateNexVal(min_name, time_sec / 60);
  updateNexVal(sec_name, time_sec % 60);
}

void displaySettings()
{
  /* Display settings */
  updateTime("ST_m.val=", "ST_s.val=", total_time_);
  updateTime("F_cycle_m.val=", "F_cycle_s.val=", first_cycle_time_);
  updateTime("P_m.val=", "P_s.val=", pause_time_);
  updateTime("R_m.val=", "R_s.val=", rep_cycle_time_);

  updateNexVal("AF.val=", A_AF);
  updateNexVal("AR.val=", A_AR);
  updateNexVal("Speed.val=", A_Speed);
}

void displayTimeLeft()
{
  updateTime("T_time_m.val=", "T_time_s.val=", time_left_ms_ / 1000);
  updateTime("C_cycle_m.val=", "C_cycle_s.val=", current_cycle_left_ms_ / 1000);
}

void handleHoldButtons()
{
  i = i + 1;
  if (up1) {
    total_time_ = min(total_time_ + increment + i, TIME_MAX);
  }
  if (up2) {
    first_cycle_time_ = min(first_cycle_time_ + increment + i, TIME_MAX);
  }
  if (up3) {
    pause_time_ = min(pause_time_ + increment + i, TIME_MAX);
  }
  if (up4) {
    rep_cycle_time_ = min(rep_cycle_time_ + increment + i, TIME_MAX);
  }

  if (up5) {
    A_AF = A_AF + increment + i;
  }
  if (up6) {
    A_AR = A_AR + increment + i;
  }

  if (up7) {
    A_Speed = A_Speed + increment + i;
    if (A_Speed >= 100) {
      A_Speed = 100;
    }
    // Serial.print("A_Speed= ");
    // Serial.println(A_Speed);
    // Serial.print("A_Speed= ");
    // Serial.println(A_Speed);
  }

  if (dw1) {
    total_time_ = max(total_time_ - increment - i, 0);
  }
  if (dw2) {
    first_cycle_time_ = max(first_cycle_time_ - increment - i, 0);
  }
  if (dw3) {
    pause_time_ = max(pause_time_ - increment - i, 0);
  }
  if (dw4) {
    rep_cycle_time_ = max(rep_cycle_time_ - increment - i, 0);
  }

  if (dw5) {
    A_AF = A_AF - increment - i;
    if (A_AF <= 0)
      A_AF = 0;
  }
  if (dw6) {
    A_AR = A_AR - increment - i;
    if (A_AR <= 0)
      A_AR = 0;
  }
  if (dw7) {
    A_Speed = A_Speed - increment - i;
    if (A_Speed <= 0)
      A_Speed = 0;
    // Serial.print("A_Speed= ");
    // Serial.println(A_Speed);
    // Serial.print("A_Speed= ");
    // Serial.println(A_Speed);
  }
}

void doMotorStep()
{
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(5);
}

// This function takes max 1 s to complete
void motorReset()
{
  const int step_delay = 5;

  if (motor.pos == 0) {
    return;
  } else if (motor.pos > 0) {
    digitalWrite(dirPin, BACKWARD);
    for (int i = 0; i < motor.pos; i++) {
      doMotorStep();
      delay(step_delay);
    }
  } else {
    digitalWrite(dirPin, FORWARD);
    for (int i = motor.pos; i < 0; i++) {
      doMotorStep();
      delay(step_delay);
    }
  }

  motor.pos = 0;
}

int calcStepDelayMicrosec(int speed_percent)
{
  long rot_per_min = MIN_ROT_PER_MINUTE;
  rot_per_min = (speed_percent - 1);
  rot_per_min *= (MAX_ROT_PER_MINUTE - MIN_ROT_PER_MINUTE);
  rot_per_min = round((rot_per_min) / (100.0 - 1.0));
  rot_per_min += MIN_ROT_PER_MINUTE;

  if (rot_per_min < MIN_ROT_PER_MINUTE) {
    rot_per_min = MIN_ROT_PER_MINUTE;
  } else if (rot_per_min > MAX_ROT_PER_MINUTE) {
    rot_per_min = MAX_ROT_PER_MINUTE;
  }

  // To calc delay in ms between every step according to speed:
  // (60 * 1000 msec/min) / ((rotation/min) * (steps/rotation)) = ms between each step
  unsigned long tmp = 60*1000LU*1000LU;
  tmp /= (rot_per_min * TB6600_STEPS_PER_REV);
  return (int)tmp;
}

// Call with only 1 ms interval
void runControl_not_blocking(unsigned long time_ms)
{
  typedef enum {FirstCycle, Pause, Repeat, Mixing, MotorReseting, DoNothing} State_e;
  static State_e state = FirstCycle;
  static State_e next_state;
  static int n_repetitions;
  static int delay_between_steps;
  static unsigned long prev_motor_step_time;

  switch (state)
  {
  case FirstCycle:
    motorReset();
    time_left_ms_ = (unsigned long)total_time_ * 1000UL;
    n_repetitions = (total_time_ - first_cycle_time_) / (pause_time_ - rep_cycle_time_);
    delay_between_steps = calcStepDelayMicrosec(A_Speed);
    motor.target_fw_pos = (int)((float)A_AF * (TB6600_STEPS_PER_REV / 360.0));
    motor.target_bw_pos = -(int)((float)A_AR * (TB6600_STEPS_PER_REV / 360.0));
    motor.current_dir = FORWARD;
    digitalWrite(dirPin, FORWARD);

    current_cycle_left_ms_ = (unsigned long)first_cycle_time_ * 1000UL;
    next_state = Pause;
    state = Mixing; // Change state to run motor
    break;
  case Pause:
    current_cycle_left_ms_ = (unsigned long)pause_time_ * 1000UL;
    next_state = Repeat;

    if (motor.pos == 0) {
      state = DoNothing;
    } else {
      if (motor.pos < 0) {
        motor.current_dir = FORWARD;
      } else {
        motor.current_dir = BACKWARD;
      }
      state = MotorReseting;
    }
    break;
  case Repeat:
    if (n_repetitions == 1) {
      current_cycle_left_ms_ = time_left_ms_;
      next_state = DoNothing;
    } else {
      current_cycle_left_ms_ = (unsigned long)rep_cycle_time_ * 1000UL;
      next_state = Pause;
    }
    n_repetitions--;
    state = Mixing;
    digitalWrite(dirPin, FORWARD);
    break;
  case Mixing:
    if ((time_ms - prev_motor_step_time) > delay_between_steps) {
      prev_motor_step_time = time_ms;
      doMotorStep();
      if (motor.current_dir == FORWARD) {
      motor.pos++;
        if (motor.pos == motor.target_fw_pos) {
          motor.current_dir = BACKWARD;
          digitalWrite(dirPin, BACKWARD);
        }
      } else {
        motor.pos--;
        if (motor.pos == motor.target_bw_pos) {
          motor.current_dir = FORWARD;
          digitalWrite(dirPin, FORWARD);
        }
      }
    }
    break;
  case MotorReseting:
    if ((time_ms - prev_motor_step_time) > delay_between_steps) {
      prev_motor_step_time = time_ms;
      doMotorStep();
      if (motor.current_dir == FORWARD) {
         motor.pos++;
      } else {
        motor.pos--;
      }
      if (motor.pos == 0) {
        state = DoNothing;
      }
    }
    break;
  case DoNothing:
    break;
  default:
    break;
  }

  current_cycle_left_ms_--;
  if (current_cycle_left_ms_ == 0)
  {
    state = next_state;
  }

  time_left_ms_--;
  if (time_left_ms_ == 0) {
    stop_shaking_ = true;
  }
}

// depricated
void runControl()
{
  delay_motion = 10; //10ms

  //==================Stepper motion==========================

  //calculate parameters------------------------------------------------------------------------------
  // TODO: Delete it later
  // A_ST = (A_ST_s + 60 * A_ST_m);                //ms
  // A_F_cycle = (A_F_cycle_s + 60 * A_F_cycle_m); //ms
  // A_P = (A_P_s + 60 * A_P_m);                   //ms
  // A_R = (A_R_s + 60 * A_R_m);                   //ms
  A_ST = total_time_;
  A_F_cycle = first_cycle_time_;
  A_P = pause_time_;
  A_R = rep_cycle_time_;

  float a = A_Speed;
  delay_motion = round((min_speed - max_speed) * (1 - a / 100) + max_speed); // calculate time of delay(ms)
  Step_F = round(A_AF * 200 / 360);                                          //N°of Steeps for Forward Angle
  Step_R = round(A_AR * 200 / 360);                                          //N°of Steeps for  Revers Angle
  Serial.println("====================================================================================");
  Serial.print("A_ST= ");
  Serial.println(A_ST);
  Serial.print("A_F_cycle= ");
  Serial.println(A_F_cycle);
  Serial.print("A_P= ");
  Serial.println(A_P);
  Serial.print("A_R= ");
  Serial.println(A_R);
  Serial.print("Step_F= ");
  Serial.println(Step_F);
  Serial.print("Step_R= ");
  Serial.println(Step_R);
  Serial.print("A_Speed= ");
  Serial.println(A_Speed);
  Serial.print("delay_motion= ");
  Serial.println(delay_motion);
  Serial.println("====================================================================================");
  //--------------------------------------------------------------------------------------------------

  //First cycle---------------------------------------
  unsigned int j = 0;
  unsigned int A_ST_m = 5;
  unsigned int A_ST_s = 0;
  unsigned int A_ST = 0;
  while (2 * delay_motion * j < A_F_cycle * 1000)
  {
    Serial.println("First cycle");
    //go to Forward Angle
    digitalWrite(dirPin, HIGH);
    for (int k = 0; k < Step_F; k++)
    {
      digitalWrite(stepPin, HIGH);
      delay(delay_motion);
      digitalWrite(stepPin, LOW);
      delay(delay_motion);
    }
    pos = pos + Step_F;
    //go to Backward Angle
    digitalWrite(dirPin, LOW);
    for (int k = 0; k < Step_F + Step_R; k++)
    {
      digitalWrite(stepPin, HIGH);
      delay(delay_motion);
      digitalWrite(stepPin, LOW);
      delay(delay_motion);
    }
    pos = pos - (Step_F + Step_R);
    //go to 0 Angle
    digitalWrite(dirPin, HIGH);
    for (int k = 0; k < Step_R; k++)
    {
      digitalWrite(stepPin, HIGH);
      delay(delay_motion);
      digitalWrite(stepPin, LOW);
      delay(delay_motion);
    }
    pos = pos + Step_R;
    j = j + 2 * (Step_F + Step_R);
    Serial.print("2*delay_motion*j=");
    Serial.println(2 * delay_motion * j);
    Serial.print("A_F_cycle=");
    Serial.println(A_F_cycle * 1000);
  }
  //-------------------------------------------------

  //Repeat cycle-------------------------------------
  r = round((A_ST - A_F_cycle) / (A_P + A_R)); //N°of Repeat
  Serial.println("-------------");
  Serial.println("-------------");
  Serial.print("A_F_cycle=");
  Serial.println(A_F_cycle);
  Serial.print("r=");
  Serial.println(r);
  for (int i = 0; i < r; i++)
  {
    Serial.println("Pause");
    delay(A_P * 1000);
    j = 0;
    Serial.println("repeat cycle");
    while (A_R * 1000 > 2 * delay_motion * j)
    {
      //go to Forward Angle
      digitalWrite(dirPin, HIGH);
      for (int k = 0; k < Step_F; k++)
      {
        digitalWrite(stepPin, HIGH);
        delay(delay_motion);
        digitalWrite(stepPin, LOW);
        delay(delay_motion);
      }
      pos = pos + Step_F;
      //go to Backward Angle
      digitalWrite(dirPin, LOW);
      for (int k = 0; k < Step_F + Step_R; k++)
      {
        digitalWrite(stepPin, HIGH);
        delay(delay_motion);
        digitalWrite(stepPin, LOW);
        delay(delay_motion);
      }
      pos = pos - (Step_F + Step_R);
      //go to 0 Angle
      digitalWrite(dirPin, HIGH);
      for (int k = 0; k < Step_R; k++)
      {
        digitalWrite(stepPin, HIGH);
        delay(delay_motion);
        digitalWrite(stepPin, LOW);
        delay(delay_motion);
      }
      pos = pos + Step_R;
      j = j + 2 * (Step_F + Step_R);
      Serial.print("A_R-2*delay_motion*j= ");
      Serial.println(A_R * 1000 - 2 * delay_motion * j);
    }
  }
}
