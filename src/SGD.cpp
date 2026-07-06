#include "wiring_constants.h"
#include "wiring_time.h"
#include <SGD.h>
#include <TimerSGD.h>

TransmitData transmitData;
SettingsData settingsData;
IncomingData incomingData;
RecievedSettingsData recievedSettingsData;

extern bool RED_alarmTriggered[8];
extern bool AMB_alarmTriggered[8];
extern GasDetection gasDetection;

void GasDetection::flashLED(int LED_PIN, int FlashDelay, int TimesToFlash) {
  for (int i = 0; i < TimesToFlash + 1; i++) {
    digitalWrite(LED_PIN, HIGH);  // Turn off the LED
    delay(FlashDelay);
    digitalWrite(LED_PIN, LOW);  // Turn off the LED
  }
};

bool GasDetection::getAlarmStatus(int SensorReadings, int ENRICHMENT, int DEPLETION) {
  bool SensorInAlam = false;

  // Check if the reading violates any threshold
  if (SensorReadings < DEPLETION || SensorReadings > ENRICHMENT) {
    return true;  // Sensor is in alarm
  }

  return SensorInAlam;
};

// Used to flash an LED based on the delay given and times given
void flashLED(int LED_PIN, int FlashDelay, int TimesToFlash) {
  for (int i = 0; i < TimesToFlash; i++) {
    digitalWrite(LED_PIN, HIGH);  // Turn LED on
    delay(FlashDelay);
    digitalWrite(LED_PIN, LOW);  // Turn LED off
    delay(FlashDelay);
  }
};

// Function to convert an Array of 8 bools to an Int
int GasDetection::bools2Int(bool boolIn[]) {
  int intOut = 0;                 // Clear all bits of intOut to 0's
  for (int i = 7; i >= 0; i--) {  //Loop backwards, i.e. from lsb to msb
    if (boolIn[i]) {
      bitSet(intOut, i);  // Set all bits to 1's
    }
  }
  return intOut;
};

// Function to convert an Int to an Array of 8 bools
void GasDetection::int2Bool(int intIn, bool boolsOut[]) {
  for (int i = 7; i >= 0; i--) {  //Loop backwards, i.e. from lsb to msb
    if (bitRead(intIn, i)) {
      boolsOut[i] = 1;  // Set all bits to 1's
    } else {
      boolsOut[i] = 0;
    }  // Clear all bits that are not 1's
  }
};

//Same logic for both of these? Condense into one possibly and call on seprate commands with different logic surronding them
void GasDetection::ResetAlarm() {
  for (int i = 0; i < 8; i++) {
    RED_alarmTriggered[i] = false;  // Reset the RED alarm flag
  }
  GasDetection::SystemStatusRedLatch = 0;  // Set the latch red alarm off
  // RESETTING SHOULD ALSO TURN OFF THE SOUNDER:
  // digitalWrite(GasDetection::SounderOutput, 0);  // Turn off physical sounder output
  GasDetection::SystemStatBoolsArr[4] = 0;       // Sounder Not Active in status array, for modbus via ESP
  GasDetection::SounderMuted = false;            // Unlike the mute function, this is not "muting", it is resetting it to a state where there are (hopefully) no alarms, so SounderMuted should be off
};

void GasDetection::MuteAlarm() {
  // digitalWrite(GasDetection::SounderOutput, 0);  // Turn off physical sounder output
  GasDetection::SystemStatBoolsArr[4] = 0;       // Sounder Not Active in status array, for modbus via ESP
  GasDetection::SounderMuted = true;             // Make a note that we are in a muted state
}

void GasDetection::reboot() {
  NVIC_SystemReset();  // Perform a software reset
};

int GasDetection::GetSensorReading(int sensorPinNumb) {
  return analogRead(sensorPinNumb);  // Return the actual sensor reading
};

void GasDetection::setupPinModes() {
  // Array of OUTPUT pins
  uint8_t outputPins[] = {
    gasDetection.LED_PIN_1,
    gasDetection.LED_PIN_2,
    gasDetection.LED_PIN_3,
    gasDetection.LED_PIN_4,
    gasDetection.Fail_Safe_Relay,
    gasDetection.RedOutput,
    gasDetection.AmberOutput,
    gasDetection.GreenOutput,
    gasDetection.SounderOutput,
    gasDetection.HealthyOutput
  };

  // Array of INPUT pins
  uint8_t inputPins[] = {
    gasDetection.RedOutputMonitorIP,
    gasDetection.AmbOutputMonitorIP,
    gasDetection.GreenOutputMonitorIP,
    gasDetection.SounderOutputMonitorIP,
    gasDetection.HealthyOutputMonitorIP,
    gasDetection.digiIn1,
    gasDetection.digiIn2,
    gasDetection.digiIn3,
    gasDetection.digiIn4,
    gasDetection.digiIn5
  };

  // Set OUTPUT pin modes
  for (uint8_t i = 0; i < sizeof(outputPins) / sizeof(outputPins[0]); i++) {
    pinMode(outputPins[i], OUTPUT);
  }

  // Set INPUT pin modes
  for (uint8_t i = 0; i < sizeof(inputPins) / sizeof(inputPins[0]); i++) {
    pinMode(inputPins[i], INPUT);
  }
}

bool GasDetection::checkCommandRisingEdge(uint8_t bitIndex) {
  bool current = bitRead(recievedSettingsData.command, bitIndex);
  bool previous = bitRead(this->lastCommand, bitIndex);

  if (current && !previous) {
    bitClear(recievedSettingsData.command, bitIndex);  // Clear so it won't retrigger
    return true;
  }
  return false;
}

void GasDetection::setFailsafeOutputs() {
  gasDetection.SystemStatBoolsArr[1] = 1;  // RED ON
  gasDetection.SystemStatBoolsArr[2] = 1;  // AMBER ON
  gasDetection.SystemStatBoolsArr[4] = 1;  // SOUNDER ON
  gasDetection.SystemStatBoolsArr[5] = 0;  // HEALTHY OFF
  gasDetection.SystemStatBoolsArr[3] = 0;  // GREEN OFF
}
