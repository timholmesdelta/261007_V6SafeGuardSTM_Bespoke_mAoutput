#include <Arduino.h>
#pragma once

class Custom_Timer {
public:
  // Sensor Level
  int systemRedAlarmDelay = 5000;  //
  int systemAmbAlarmDelay = 1000;  //

  unsigned long InitialiseStartTime;
  unsigned long InitialiseDelay = 10000;  // 10 seconds in milliseconds

  //System Level
  unsigned long A2D_StartTime;
  unsigned long A2D_Delay = 356400000;  // 99 Hours in milliseconds

  unsigned long UnderRangeStartTime;
  unsigned long UnderRangeDelay = 120000;

  unsigned long ExternalStartTime;       
  unsigned long ExternalFaultDelay = 600000;  // 10 minutes in milliseconds

  unsigned long OutputFaultStartTime;     
  unsigned long OutputFaultDelay = 3000;  // 3 seconds in milliseconds

  // Requires setting up with correct data
  // unsigned long CommsToSlaveTime = 0;
  // unsigned long CommsToSlaveDelay = 0; // Requires setting up

  bool TimerElapsed(unsigned long TmInterval, unsigned long TmStart);
  bool TimerElapsedInitialise(unsigned long TmInterval, unsigned long TmStart);
  long StartTimer();
};
