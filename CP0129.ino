
/*
CP0129  Copyright Cyrob 2022
Cyrob Adjustable Timed Relay by Philippe Demerliac
See my presentation video in French : https://youtu.be/PND29Onvhac
=====================================================================================
==========================   OPEN SOURCE LICENCE   ==================================
=====================================================================================
Copyright 2021 Philippe Demerliac Cyrob.org
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
................................................................................................................
Release history
................................................................................................................
Version Date        Author    Comment
1.0     19/05/2022 Phildem   First blood

*/


//____________________________________________________________________________________________________________
// Includes __________________________________________________________________________________

#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

// CP0129 is base on C.A.T.H task supervisor see https://github.com/Phildem/Cath
//____________________________________________________________________________________________________________
// Start of Cath definition__________________________________________________________________________________
 
#define kMaxCathTask    6         // Max Number of task instances. MUST BE >= to tasks instancied

#define __CathOpt_SmallCounter__  // Comment this line to allow 32 bit delay. If not, max period is 65536 ms

#ifdef __CathOpt_SmallCounter__
typedef uint16_t CathCnt;
#else
typedef uint32_t CathCnt;
#endif


class Cath {

  public:

// Derived class MUST implement these 2 methods
  virtual void          SetUp() =0;                 // Called at setup
  virtual void          Loop()  =0;                 // Called periodically

  CathCnt               m_CurCounter;               // Curent number of ms before next Loop call
  CathCnt               m_LoopDelay;                // Default period of Loop call (in ms)

  static uint8_t        S_NbTask;                   // Actual number of task instances
  static Cath*          S_CathTasks[kMaxCathTask];  // Array of task object pointers
  static uint8_t        S_LastMilli;                // Used to call every ms (a byte is enought to detect change)

  //..............................................................
  // Must be called in task constructors to register in the task list
  // WARNING : think to set kMaxCathTask as needed
  // Task   : Pointer to the derivated task to register
  // Period : Loop call Period (in ms). WARNING do not pass 0!
  // Offset : Delay of the first call in ms (1 def). WARNING do not pass 0!
  static void S_Register(Cath* Task,CathCnt Period,CathCnt Offset=1){
    Task->m_LoopDelay=Period;
    Task->m_CurCounter= Offset;
    Cath::S_CathTasks[Cath::S_NbTask++]=Task;
  }

  //..............................................................
  // Must be called once in Arduino setup to call all the task setups
  static void S_SetUp(){
    for(int T=0;T<S_NbTask;T++)
      Cath::S_CathTasks[T]->SetUp();
  }

   //..............................................................
  // Must be called once in Arduino Loop to call all the task loop if needed
  static void S_Loop(){
    uint8_t CurMilli=millis();
    if (CurMilli!=S_LastMilli) {
      S_LastMilli=CurMilli;
      for(int T=0;T<S_NbTask;T++) 
        if ( Cath::S_CathTasks[T]->m_CurCounter--==0) {
          Cath::S_CathTasks[T]->m_CurCounter=Cath::S_CathTasks[T]->m_LoopDelay;
          Cath::S_CathTasks[T]->Loop();
        }
     }
  }

};

//Cath static variables definitions 
//(Note set to 0 for code clarity but done by default anyway because they are static)
uint8_t       Cath::S_NbTask=0;
Cath*         Cath::S_CathTasks[kMaxCathTask];
uint8_t       Cath::S_LastMilli=0;

// End of Cath definition ___________________________________________________________________________________
//___________________________________________________________________________________________________________


//****************************************************************************************************************
// I/O Abstraction
#define kOutPinLedOn              2         // Ooutput On led Active high
#define kOutPinShutDown           3         // "Suicide" output Active high
#define kOutPinLoad               4         // Load relay Active high

#define kInPinLoadCtl             5         // Analog input for time setting

#define kInPinAdPot               A0        // Analog input for time setting

//****************************************************************************************************************
// Misc Constants

// Global -------------------------------------------------------------------
#define kTaskShift                23        // Task phase shifter

// Display  -----------------------------------------------------------------
#define kDisplayTaskPeriod        250       // Display task period in ms

#define kDisplayI2CAdress         0x70      // Display module Adress

#define kDisplaySplashDuration    1000      // 88:88 Test display at startup duration in ms

// ClockTask ----------------------------------------------------------------
#define kClockTaskPeriod          1000      // ClockTask task period in ms

// TimeCtl ------------------------------------------------------------------

/*
ATTENTION, important point about the time setting :

The A/D converter of the Arduino have a 10bits resolution and returns a value from 0 to 1023.

If we want a maximum time of 60mn, that is 3600 seconds, we understand that it is impossible to have a resolution of one second, it could at best be 3600/1023 or 3.5 seconds which is not very practical.
I chose 30 seconds for my use but 10 would have been possible.

It is therefore obvious that depending on the min/max setting values chosen, this constraint must be taken into account.

For the moment, the program does not display times greater than 99 minutes.
If you want longer, you can of course modify the program to switch to hh:mm for times longer than 1 hour.
*/

#define kTimeCtlTaskPeriod        250       // TimeCtl task period in ms

#define kTimeCtlMinDuration       30        // Minimum On duration in second
#define kTimeCtlMaxDuration       (60*60)   // Maximum On duration in second
#define kTimeCtlResolution        30        // Setting Resolution in Sec

// LoadCtl ------------------------------------------------------------------
#define kLoadCtlTaskPeriod        30        // LoadCtl task period in ms

// LoadDisplay --------------------------------------------------------------
#define kLoadDisplayTaskPeriod    1000      // LoadDisplay task period in ms

// AutoShutDown -------------------------------------------------------------
#define kAutoShutDownTaskPeriod   1000      // AutoShutDown task period in ms

#define kAutoShutDownDelaySec     30        // Delay before Shutdown in sec
#define kAutoShutDownTol          20        // Min Time change to delay shutdown

//****************************************************************************************************************
// Globals
bool            gLoadOn;                    //  True if load On
unsigned long   gDurationSec;               //  Duration in Sec Wanted if LoadOff, current if loadOn


//Tasks___________________________________________________________________________________________________________



//Clock ..........................................................................................
class Clock: public Cath {

  public:

  //..............................................................
  Clock() {
    Cath::S_Register(this,kClockTaskPeriod,kTaskShift*Cath::S_NbTask);
  }
  //..............................................................
  void SetUp() {

  }

  //..............................................................
  void Loop(){

    if (gLoadOn==true && gDurationSec>0){    // Countdown if load id on
      gDurationSec--;
      if (gDurationSec==0)    // If time elapsed, return to Iddle Mode
        gLoadOn=false;
    }
  }

};

//TimeCtl ..........................................................................................
class TimeCtl: public Cath {

  public:

  //..............................................................
  TimeCtl() {
    Cath::S_Register(this,kTimeCtlTaskPeriod,kTaskShift*Cath::S_NbTask);
  }
  //..............................................................
  void SetUp() {

  }

  //..............................................................
  void Loop(){

    if (gLoadOn==true) // Do Nothing if load is on
      return;
    
    unsigned long  Pot=analogRead(kInPinAdPot);
    
    gDurationSec=map(Pot,0,1023,kTimeCtlMinDuration,kTimeCtlMaxDuration);
    gDurationSec=map(gDurationSec,kTimeCtlMinDuration,kTimeCtlMaxDuration,kTimeCtlMinDuration/kTimeCtlResolution,kTimeCtlMaxDuration/kTimeCtlResolution);
    gDurationSec=gDurationSec*kTimeCtlResolution;
  }

};

//LoadCtl ..........................................................................................
class LoadCtl: public Cath {

  public:

  //..............................................................
  LoadCtl() {
    m_LastButtonState=true;
    Cath::S_Register(this,kLoadCtlTaskPeriod,kTaskShift*Cath::S_NbTask);
  }
  //..............................................................
  void SetUp() {
    pinMode(kInPinLoadCtl,INPUT_PULLUP);
    pinMode(kOutPinLoad,OUTPUT);
    digitalWrite(kOutPinLoad,LOW);
  }

  //..............................................................
  void Loop(){

    bool ButtonState=digitalRead(kInPinLoadCtl);
    if (ButtonState!=m_LastButtonState){
      m_LastButtonState=ButtonState;
      if (ButtonState==false) {   // Only push down is handled
          gLoadOn=!gLoadOn;         // Toggle Load 
      }
    }

    digitalWrite(kOutPinLoad,gLoadOn);  // Command load vs gLoadOn
  }

  bool m_LastButtonState;       // Last button state, used to check change
};


//LoadDisplay ..........................................................................................
class LoadDisplay: public Cath {

  public:

  //..............................................................
  LoadDisplay() {
    Cath::S_Register(this,kLoadDisplayTaskPeriod,kTaskShift*Cath::S_NbTask);
  }
  //..............................................................
  void SetUp() {
    pinMode(kOutPinLedOn,OUTPUT);
    digitalWrite(kOutPinLedOn,LOW);
  }

  //..............................................................
  void Loop(){
    if (gLoadOn==true)
      digitalWrite(kOutPinLedOn,!digitalRead(kOutPinLedOn));
    else
      digitalWrite(kOutPinLedOn,LOW);
  }

};

//AutoShutDown ..........................................................................................
class AutoShutDown: public Cath {

  public:

  //..............................................................
  AutoShutDown() {
    m_LastgDurationSec=0;
    m_InactivityCounter=0;
    Cath::S_Register(this,kAutoShutDownTaskPeriod,kTaskShift*Cath::S_NbTask);
  }
  //..............................................................
  void SetUp() {
    pinMode(kOutPinShutDown,OUTPUT);
    digitalWrite(kOutPinShutDown,LOW);
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
  }

  //..............................................................
  void Loop(){

    // Look if gDurationSec has changed
    if ((gLoadOn==true) || (gDurationSec<(m_LastgDurationSec-kAutoShutDownTol)) || (gDurationSec>(m_LastgDurationSec+kAutoShutDownTol))){
      m_LastgDurationSec=constrain(gDurationSec,kAutoShutDownTol,kTimeCtlMaxDuration-kAutoShutDownTol);
      m_InactivityCounter=0;
    }
    else {
      digitalWrite(LED_BUILTIN,HIGH);
      m_InactivityCounter++;
      if (m_InactivityCounter>=kAutoShutDownDelaySec)
      {
        digitalWrite(kOutPinShutDown,HIGH);
        delay(5000);
      }
      else {
        delay(20);
        digitalWrite(LED_BUILTIN,LOW);
      }  
    }

  }

  unsigned long   m_LastgDurationSec;
  uint8_t         m_InactivityCounter;
};

//Display ..........................................................................................
class DisplayCtl: public Cath{

  public:

  //..............................................................
  DisplayCtl() {

    Cath::S_Register(this,kDisplayTaskPeriod,kTaskShift*Cath::S_NbTask);
  }

  //..............................................................
  void SetUp() {

      m_Display.begin(kDisplayI2CAdress);
  
      m_Display.print(8888, DEC);
      m_Display.drawColon(true);
      m_Display.writeDisplay();
      delay(kDisplaySplashDuration);
  }

  //..............................................................
  void Loop(){

    unsigned long Disp=gDurationSec;

    Disp=(Disp/60)*100+Disp%60;
    
    m_Display.print(Disp, DEC);
    m_Display.drawColon(true);
   
    m_Display.writeDisplay();
  }


Adafruit_7segment m_Display = Adafruit_7segment();

};



//****************************************************************************************************************
// Global tasks instanciation
// **** WARNING TASK ORDER MUST BE RESPECTED ****

// 1 Instance of the Clock to manage time
Clock         gClock;

// 1 Instance of the Clock to manage time Setting
TimeCtl       gTimeCtl;

// 1 Instance of the LoadCtl to manage Load On/Off
LoadCtl       gLoadCtl;

// 1 Instance of the LoadDisplay to manage Display of load state
LoadDisplay   gLoadDisplay;

// 1 Instance of the AutoShutDown to manage AutoShutDown when not used
AutoShutDown  gAutoShutDown;

// 1 instance of DisplayCtl to control Time display
DisplayCtl    gDisplayCtl;


//-----------------------------------------------------------------------------------------------------------------
void setup() {
  Cath::S_SetUp();    // Just ask Cath to call the task's setup
}

//-----------------------------------------------------------------------------------------------------------------
void loop() {
  Cath::S_Loop();    // Just ask Cath to call the task's loop
}
