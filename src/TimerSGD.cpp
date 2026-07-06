#include <TimerSGD.h>

bool Custom_Timer::TimerElapsed(unsigned long TmInterval, unsigned long TmStart) {
  unsigned long now = millis();  // now: timestamp
  unsigned long elapsed = now - TmStart;
  if (TmStart == 0) {
    return false;
  } else {
    return (elapsed >= TmInterval);  // elapsed: duration
  }
}

bool Custom_Timer::TimerElapsedInitialise(unsigned long TmInterval, unsigned long TmStart) {
  unsigned long now = millis();  // now: timestamp
  unsigned long elapsed = now - TmStart;
  return (elapsed >= TmInterval);  // elapsed: duration
}

long Custom_Timer::StartTimer() {
  return millis();
}