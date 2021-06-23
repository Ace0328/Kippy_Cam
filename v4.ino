#include <math.h>

#include <doxygen.h>
#include <NexButton.h>
#include <NexCheckbox.h>
#include <NexConfig.h>
#include <NexCrop.h>
#include <NexDualStateButton.h>
#include <NexGauge.h>
#include <NexGpio.h>
#include <NexHardware.h>
#include <NexHotspot.h>
#include <NexNumber.h>
#include <NexObject.h>
#include <NexPage.h>
#include <NexPicture.h>
#include <NexProgressBar.h>
#include <NexRadio.h>
#include <NexRtc.h>
#include <NexScrolltext.h>
#include <NexSlider.h>
#include <NexText.h>
#include <NexTimer.h>
#include <Nextion.h>
#include <NexTouch.h>
#include <NexUpload.h>
#include <NexVariable.h>
#include <NexWaveform.h>


#include <Nextion.h>

#define dirPin 12
#define stepPin 13

#define TIME_MAX (3599)

#define PRESET_TOTAL_TIME       (10 * 60) // 10 minutes = 600 seconds
#define PRESET_FIRST_CYCLE_TIME (30)      // 30 seconds
#define PRESET_PAUSE_TIME       (50)      // 50 seconds
#define PRESET_REP_CYCLE_TIME   (10)      // 10 seconds

int total_time_         = PRESET_TOTAL_TIME;
int first_cycle_time_   = PRESET_FIRST_CYCLE_TIME;
int pause_time_         = PRESET_PAUSE_TIME;
int rep_cycle_time_     = PRESET_REP_CYCLE_TIME;
int current_cycle_time_ = 0;

int max_speed=3;//delay(1ms faster speed)
int min_speed=15;//delay(500ms slower speed)

unsigned int delay_motion=0;//delay for 1 step ms
bool A_Start = false;
bool A_Stop = false;
bool A_Reset = false;
bool A_Left = false;
bool A_Right = false;

unsigned int  A_ST = 0;
unsigned int  A_CT = 0;
unsigned int A_F_cycle = 0;
unsigned int A_P = 0;
unsigned int A_R = 0;

//preset value
unsigned int A_AF = 185;
unsigned int A_AR = 80;

//preset value
int A_Speed = 85;

int increment=1;
int time_click=5;//wait 5ms
int i=0;
int pos=0;
unsigned int Step_F=0;
unsigned int Step_R=0;
unsigned int r=0;

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

//---------Start button-------------------------------------------------
void StartPushCallback(void *ptr)  // Press event for button Start
{
  A_Start = true;
  A_Stop = false;
}  // End of press event


//---------Stop button-------------------------------------------------
void StopPushCallback(void *ptr)  // Press event for button Stop
{
  A_Stop = true;
  A_Start = false;
}  // End of press event


//---------Reset button-------------------------------------------------
void ResetPushCallback(void *ptr)  // Press event for button Reset
{
  A_Reset = true;
}  // End of press event

void ResetPopCallback(void *ptr)  // Press event for button UP
{
  A_Reset=false;
}


//---------Left button-------------------------------------------------
void LeftPushCallback(void *ptr)  // Press event for button Left
{
  A_Left = true;
  A_Right = false;
}
void LeftPopCallback(void *ptr)  // Press event for button UP
{
  A_Left = false;
} // End of press event

//---------Right button-------------------------------------------------
void RightPushCallback(void *ptr)  // Press event for button Right
{
  A_Right = true;
  A_Left = false;
}
void RightPopCallback(void *ptr)  // Press event for button UP
{
  A_Right = false;
}// End of press event


//---------Total time-------------------------------------------------
void UP1PushCallback(void *ptr)  // Press event for button UP1
{
  total_time_ = min(total_time_ + increment, TIME_MAX);
  delay(time_click);
  i=0;
  up1=true;
}

void UP1PopCallback(void *ptr)  // Press event for button UP
{
  up1=false;
}

void DW1PushCallback(void *ptr)  // Press event for button DW1
{
  total_time_ = max(total_time_ - increment, 0);
  delay(time_click);
  i=0;
  dw1=true;
}
void DW1PopCallback(void *ptr)  // Press event for button DW
{
  dw1=false;
}

//---------First cycle-------------------------------------------------
void UP2PushCallback(void *ptr)  // Press event for button UP2
{
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
void UP5PushCallback(void *ptr)  // Press event for button UP5
{
  A_AF = A_AF + increment;
  delay(time_click);
  i=0;
  up5=true;
}  // End of press event

void UP5PopCallback(void *ptr)  // Press event for button UP
{
  up5=false;
}
void DW5PushCallback(void *ptr)  // Press event for button DW5
{
  A_AF = A_AF - increment;
  if(A_AF<=0)A_AF=0;
  delay(time_click);
  i=0;
  dw5=true;
}
void DW5PopCallback(void *ptr)  // Press event for button DW
{
  dw5=false;
}// End of press event


//---------Angle REVERS-------------------------------------------------
void UP6PushCallback(void *ptr)  // Press event for button UP6
{
  A_AR = A_AR + increment;
  delay(time_click);
  i=0;
  up6=true;
}  // End of press event
void UP6PopCallback(void *ptr)  // Press event for button UP
{
  up6=false;
}
void DW6PushCallback(void *ptr)  // Press event for button DW6
{
  A_AR = A_AR - increment;
  if(A_AR<=0)A_AR=0;
  delay(time_click);
  i=0;
  dw6=true;
}
void DW6PopCallback(void *ptr)  // Press event for button DW
{
  dw6=false;
}// End of press event

//---------Speed-------------------------------------------------
void UP7PushCallback(void *ptr)  // Press event for button UP7
{
  A_Speed = A_Speed + increment;
  if(A_Speed>=100)
  A_Speed=100;
  Serial.println("");
  Serial.println("========================");
  up7=true;
  delay(time_click);
  i=0;
}  // End of press event

void UP7PopCallback(void *ptr)  // Press event for button UP7
{
  up7=false;
}
void DW7PushCallback(void *ptr)  // Press event for button DW7
{
  A_Speed = A_Speed - increment;
  if(A_Speed<=0)
  A_Speed=0;
  delay(time_click);
  i=0;
  dw7=true;
}
void DW7PopCallback(void *ptr)  // Press event for button DW
{
  dw7=false;
}// End of press event

void setup() {

  Serial.begin(9600);  // Start serial comunication at baud=9600
 // delay(500);  // This dalay is just in case the nextion display didn't start yet, to be sure it will receive the following command.
//  Serial.print("baud=115200");  // Set new baud rate of nextion to 115200, but it's temporal. Next time nextion is power on,
//  Serial.write(0xff);  // We always have to send this three lines after each command sent to nextion.
//  Serial.write(0xff);
//  Serial.write(0xff);
//  Serial.end();  // End the serial comunication of baud=9600
 // Serial.begin(115200);  // Start serial comunication at baud=115200
nexInit();
 // Register the event callback functions of each touch event:
  Start.attachPush(StartPushCallback, &Start);  // Button press
  Stop.attachPush(StopPushCallback, &Stop);  // Button press
  Reset.attachPush(ResetPushCallback, &Reset);  // Button press
  Reset.attachPop(ResetPopCallback, &Reset);  // Button press
  Left.attachPush(LeftPushCallback, &Left);  // Button press
  Left.attachPop(LeftPopCallback, &Left);  // Button press
  Right.attachPush(RightPushCallback, &Right);  // Button press
  Right.attachPop(RightPopCallback, &Right);  // Button press

  UP1.attachPush(UP1PushCallback, &UP1);  // Button press
  UP2.attachPush(UP2PushCallback, &UP2);  // Button press
  UP3.attachPush(UP3PushCallback, &UP3);  // Button press
  UP4.attachPush(UP4PushCallback, &UP4);  // Button press
  UP5.attachPush(UP5PushCallback, &UP5);  // Button press
  UP6.attachPush(UP6PushCallback, &UP6);  // Button press
  UP7.attachPush(UP7PushCallback, &UP7);  // Button press
  UP1.attachPop(UP1PopCallback, &UP1);  // Button press
  UP2.attachPop(UP2PopCallback, &UP2);  // Button press
  UP3.attachPop(UP3PopCallback, &UP3);  // Button press
  UP4.attachPop(UP4PopCallback, &UP4);  // Button press
  UP5.attachPop(UP5PopCallback, &UP5);  // Button press
  UP6.attachPop(UP6PopCallback, &UP6);  // Button press
  UP7.attachPop(UP7PopCallback, &UP7);  // Button press

  DW1.attachPush(DW1PushCallback, &DW1);  // Button press
  DW2.attachPush(DW2PushCallback, &DW2);  // Button press
  DW3.attachPush(DW3PushCallback, &DW3);  // Button press
  DW4.attachPush(DW4PushCallback, &DW4);  // Button press
  DW5.attachPush(DW5PushCallback, &DW5);  // Button press
  DW6.attachPush(DW6PushCallback, &DW6);  // Button press
  DW7.attachPush(DW7PushCallback, &DW7);  // Button press
  DW1.attachPop(DW1PopCallback, &DW1);  // Button press
  DW2.attachPop(DW2PopCallback, &DW2);  // Button press
  DW3.attachPop(DW3PopCallback, &DW3);  // Button press
  DW4.attachPop(DW4PopCallback, &DW4);  // Button press
  DW5.attachPop(DW5PopCallback, &DW5);  // Button press
  DW6.attachPop(DW6PopCallback, &DW6);  // Button press
  DW7.attachPop(DW7PopCallback, &DW7);  // Button press

  // End of registering the event callback functions

    // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
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

void loop() {
  delay(30);  // This is the only delay on this loop.

//----Display Total time--------------------
  updateTime("T_time_m.val=", "T_time_s.val=", total_time_);

  // TODO: Display current cycle time

  //----Display Total time--------------------
  updateTime("ST_m.val=", "ST_s.val=", total_time_);

//----Display First cycle time--------------------
  updateTime("F_cycle_m.val=", "F_cycle_s.val=", first_cycle_time_);

//----Display Pause time--------------------
  updateTime("P_m.val=", "P_s.val=", pause_time_);

//----Display Repeate cycle time--------------------
  updateTime("R_m.val=", "R_s.val=", rep_cycle_time_);

//----Display Angle Forward--------------------
  updateNexVal("AF.val=", A_AF);

//----Display Angle Reverce--------------------
  updateNexVal("AR.val=", A_AR);

//----Display Speed--------------------
  updateNexVal("Speed.val=", A_Speed);

  nexLoop(nex_listen_list); // Check for any touch event

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
    Serial.print("A_Speed= ");
    Serial.println(A_Speed);
    Serial.print("A_Speed= ");
    Serial.println(A_Speed);
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
    Serial.print("A_Speed= ");
    Serial.println(A_Speed);
    Serial.print("A_Speed= ");
    Serial.println(A_Speed);
  }

  delay_motion = 10; //10ms
  if (A_Start)
  {
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
      delay(A_P*1000);
      j=0;
      Serial.println("repeat cycle");
      while(A_R*1000>2*delay_motion*j)
      {
       //go to Forward Angle
        digitalWrite(dirPin, HIGH);
        for(int k=0;k<Step_F;k++){
          digitalWrite(stepPin, HIGH);
          delay(delay_motion);
          digitalWrite(stepPin, LOW);
          delay(delay_motion);
        }
        pos=pos+Step_F;
        //go to Backward Angle
        digitalWrite(dirPin, LOW);
        for(int k=0;k<Step_F+Step_R;k++){
          digitalWrite(stepPin, HIGH);
          delay(delay_motion);
          digitalWrite(stepPin, LOW);
          delay(delay_motion);
        }
        pos=pos-(Step_F+Step_R);
        //go to 0 Angle
        digitalWrite(dirPin, HIGH);
        for(int k=0;k<Step_R;k++){
          digitalWrite(stepPin, HIGH);
          delay(delay_motion);
          digitalWrite(stepPin, LOW);
          delay(delay_motion);
        }
        pos=pos+Step_R;
        j=j+2*(Step_F+Step_R);
         Serial.print("A_R-2*delay_motion*j= ");  Serial.println(A_R*1000-2*delay_motion*j);
      }
  }
A_Start=false;
  } //end Start

    if(A_Left)digitalWrite(dirPin, LOW);
    while (A_Left){
          digitalWrite(stepPin, HIGH);
          delay(delay_motion*10);
          nexLoop(nex_listen_list);  // Check for any touch event

          digitalWrite(stepPin, LOW);
          delay(delay_motion*10);
          nexLoop(nex_listen_list);  // Check for any touch event
}
    if(A_Right)digitalWrite(dirPin, HIGH);
    while (A_Right){
          digitalWrite(stepPin, HIGH);
          delay(delay_motion*10);
         nexLoop(nex_listen_list);  // Check for any touch event

          digitalWrite(stepPin, LOW);
          delay(delay_motion*10);
        nexLoop(nex_listen_list);  // Check for any touch event
}
}

