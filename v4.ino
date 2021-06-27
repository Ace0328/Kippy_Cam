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
int calcStepDelayMicrosec(int speed_percent);

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
    while (pos_ != 0) {
      doStep();
      delayMicroseconds(delay_us);
    }
    setDir(FORWARD);
  }

  void changeDir() {
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
    , running_(false)
  {
    motor_ptr_ = m;

    time_left_us_ = (unsigned long) s.total_time * 1000UL;
    n_repetitions_ = (s.total_time - s.first_cycle_time)
                    / (s.pause_time - s.rep_cycle_time);
    delay_between_steps_us_ = calcStepDelayMicrosec(s.speed);

    target_fw_pos_ = (int)((float)s.angle_forward * (TB6600_STEPS_PER_REV / 360.0));
    target_bw_pos_ = -(int)((float)s.angle_backward * (TB6600_STEPS_PER_REV / 360.0));

    settings_ = s;
  }

  void runLoop() {

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

  void Stop() {
    running_ = false;
    time_left_us_ = 0;
    current_cycle_left_us_ = 0;
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
Settings settings_;
int i=0;

/* Counters to call function with some periodic interval */
unsigned long prev_millis_for_GUI = 0;
unsigned long prev_millis_for_print = 0;

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

  mixer = Mixer(settings_, &motor_new);
}

void loop()
{
  if (mixer.isRunning()) {
    mixer.runLoop();

    if (stop_shaking_) {
      stop_shaking_ = false;
      mixer.Stop();
      displayTimeLeft();
      motor_new.reset(calcStepDelayMicrosec(settings_.speed));
    }
  }

  // Check for any touch event
  nexLoop(nex_listen_list);

  // Read GUI and handle buttons every TIM_MS_INT_GUI (1000/TIM_MS_INT_GUI per sec)
  if ((millis() - prev_millis_for_GUI) >= TIM_MS_INT_GUI) {
    prev_millis_for_GUI = millis();

    if (!mixer.isRunning()) {
      handleHoldButtons();
      // Turn left if the button is held
      int delay_motion = calcStepDelayMicrosec(20);
      if (A_Left) {
        digitalWrite(dirPin, LOW);
      }
      while (A_Left) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(delay_motion);
        nexLoop(nex_listen_list); // Check for any touch event

        digitalWrite(stepPin, LOW);
        delayMicroseconds(delay_motion);
        nexLoop(nex_listen_list); // Check for any touch event
      }

      // Turn right if the button is held
      if (A_Right) {
        digitalWrite(dirPin, HIGH);
      }
      while (A_Right) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(delay_motion);
        nexLoop(nex_listen_list); // Check for any touch event

        digitalWrite(stepPin, LOW);
        delayMicroseconds(delay_motion);
        nexLoop(nex_listen_list); // Check for any touch event
      }
    }
  }

  // Update values evert TIM_MS_INT_PRINT ms
  if ((millis() - prev_millis_for_print) >= TIM_MS_INT_PRINT) {
    prev_millis_for_print = millis();
    if (!mixer.isRunning()) {
      displaySettings();
    } else {
      displayTimeLeft();
    }
  }

  if (start_shaking_) {
    // TODO: Verify all time settings before running
    start_shaking_ = false;
    mixer = Mixer(settings_, &motor_new);
    mixer.Start();
  }
}

void resetSettingsToDefault()
{
  settings_.total_time = PRESET_TOTAL_TIME;
  settings_.first_cycle_time = PRESET_FIRST_CYCLE_TIME;
  settings_.pause_time = PRESET_PAUSE_TIME;
  settings_.pause_time = PRESET_REP_CYCLE_TIME;
  settings_.angle_forward = PRESET_ANGLE_FW;
  settings_.angle_backward = PRESET_ANGLE_BW;
  settings_.speed = PRESET_SPEED;
}

//---------Start button-------------------------------------------------
void StartPushCallback(void *ptr)
{
  if (mixer.isRunning()) {
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
  if (mixer.isRunning()) {
    return;
}
  resetSettingsToDefault();
  motor_new.reset(calcStepDelayMicrosec(settings_.speed));

  displaySettings();
  displayTimeLeft();
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
  if (mixer.isRunning()) {
    return;
  }
  settings_.total_time = min(settings_.total_time + increment, TIME_MAX);
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
  if (mixer.isRunning()) {
    return;
  }
  settings_.total_time = max(settings_.total_time - increment, 0);
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
  if (mixer.isRunning()) {
    return;
  }
  // TODO: check if total time became more that TIME_MAX
  settings_.first_cycle_time = min(settings_.first_cycle_time + increment,  TIME_MAX);
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
  if (mixer.isRunning()) {
    return;
  }
  settings_.first_cycle_time = max(settings_.first_cycle_time - increment, 0);
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
  if (mixer.isRunning()) {
    return;
  }
  // TODO: check if total time became more that TIME_MAX
  settings_.pause_time = min(settings_.pause_time + increment, TIME_MAX);
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
  if (mixer.isRunning()) {
    return;
  }
  settings_.pause_time = max(settings_.pause_time - increment, 0);
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
  if (mixer.isRunning()) {
    return;
  }
  // TODO: check if total time became more that TIME_MAX
  settings_.rep_cycle_time = min(settings_.rep_cycle_time + increment, TIME_MAX);
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
  if (mixer.isRunning()) {
    return;
  }
  settings_.rep_cycle_time = max(settings_.rep_cycle_time - increment, 0);
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
  if (mixer.isRunning()) {
    return;
  }
  settings_.angle_forward += increment;
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
  if (mixer.isRunning()) {
    return;
  }
  settings_.angle_forward = max(settings_.angle_forward - increment, 0);
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
  if (mixer.isRunning()) {
    return;
  }
  settings_.angle_backward += increment;
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
  if (mixer.isRunning()) {
    return;
  }
  settings_.angle_backward = max(settings_.angle_backward - increment, 0);
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
  if (mixer.isRunning()) {
    return;
  }
  settings_.speed = min(settings_.speed + increment, 100);
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
  if (mixer.isRunning()) {
    return;
  }
  settings_.speed = max(settings_.speed - increment, 0);
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
  updateTime("ST_m.val=", "ST_s.val=", settings_.total_time);
  updateTime("F_cycle_m.val=", "F_cycle_s.val=", settings_.first_cycle_time);
  updateTime("P_m.val=", "P_s.val=", settings_.pause_time);
  updateTime("R_m.val=", "R_s.val=", settings_.rep_cycle_time);

  updateNexVal("AF.val=", settings_.angle_forward);
  updateNexVal("AR.val=", settings_.angle_backward);
  updateNexVal("Speed.val=", settings_.speed);
}

void displayTimeLeft()
{
  updateTime("T_time_m.val=", "T_time_s.val=", mixer.timeLeft());
  updateTime("C_cycle_m.val=", "C_cycle_s.val=", mixer.timeLeftCycle());
}

void handleHoldButtons()
{
  i = i + 1;
  if (up1) {
    settings_.total_time = min(settings_.total_time + increment + i, TIME_MAX);
  }
  if (up2) {
    settings_.first_cycle_time = min(settings_.first_cycle_time + increment + i, TIME_MAX);
  }
  if (up3) {
    settings_.pause_time = min(settings_.pause_time + increment + i, TIME_MAX);
  }
  if (up4) {
    settings_.rep_cycle_time = min(settings_.rep_cycle_time + increment + i, TIME_MAX);
  }

  if (up5) {
    settings_.angle_forward += (increment + i);
  }
  if (up6) {
    settings_.angle_backward += (increment + i);
  }

  if (up7) {
    settings_.speed = min(settings_.speed + increment + i, 100);
  }

  if (dw1) {
    settings_.total_time = max(settings_.total_time - increment - i, 0);
  }
  if (dw2) {
    settings_.first_cycle_time = max(settings_.first_cycle_time - increment - i, 0);
  }
  if (dw3) {
    settings_.pause_time = max(settings_.pause_time - increment - i, 0);
  }
  if (dw4) {
    settings_.rep_cycle_time = max(settings_.rep_cycle_time - increment - i, 0);
  }

  if (dw5) {
    settings_.angle_forward = max(settings_.angle_forward - increment - i, 0);
  }
  if (dw6) {
    settings_.angle_backward = max(settings_.angle_backward - increment - i, 0);
  }
  if (dw7) {
    settings_.speed = max(settings_.speed - increment - i, 0);
  }
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
