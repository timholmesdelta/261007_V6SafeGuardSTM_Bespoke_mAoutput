#ifndef TRANSMIT_DATA_H
#define TRANSMIT_DATA_H

#include "Arduino.h"  // because serial doesnt work without it in the .cpp file
#include <stdint.h>

class GasDetection {
public:
  bool getAlarmStatus(int SensorReadings, int ENRICHMENT, int DEPLETION);  // Gets current alarm Status
  void ResetAlarm();                                                       // Resets any current Alarm
  void MuteAlarm();                                                        // Resets any current Alarm
  void flashLED(int LED_PIN, int FlashDelay, int TimesToFlash);
  int bools2Int(bool boolIn[]);               // pass in a Bool array (length must be 8) and function returns it converted to an int
  void int2Bool(int intIn, bool boolsOut[]);  // pass in an int for conversion to an array of bools (length must be [8]) and replaces the array argument values passed in
  int GetSensorReading(int sensorPinNumb);    //Gets a sensor reading based on the pin provided
  void setupPinModes();
  bool checkCommandRisingEdge(uint8_t bitIndex);
  void setFailsafeOutputs();
  const int ADC_20mA = 748;                   // raw input DAC Value at 20ma
  const int ADC_4mA = 149;                   // raw input DAC Value at 20ma
  void reboot();
  bool SounderMuted = false;
  bool SystemStatBoolsArr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  bool SystemFaultBoolsArr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  bool SystemDigiInputs[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  bool SystemOutputs[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

  
  byte lastCommand = 0;
  bool SystemStatusRedLatch = false;

  // Relays
  uint8_t Fail_Safe_Relay = PA8;
  uint8_t RedOutput = PC9;
  uint8_t AmberOutput = PC8;
  uint8_t GreenOutput = PC7;
  uint8_t SounderOutput = PC6;
  uint8_t HealthyOutput = PD15;

  // LEDs
  uint8_t LED_PIN_1 = PB14;  // select the pin for the LED
  uint8_t LED_PIN_2 = PB13;
  uint8_t LED_PIN_3 = PB12;
  uint8_t LED_PIN_4 = PB15;

  // These are digital inputs for monitoring the state of physical outputs
  uint8_t RedOutputMonitorIP = PE14; 
  uint8_t AmbOutputMonitorIP = PB10; 
  uint8_t GreenOutputMonitorIP = PE13;
  uint8_t SounderOutputMonitorIP = PE15;
  uint8_t HealthyOutputMonitorIP = PE12;

  uint8_t AnalogueOutput = PA9; 
  
  // General Purpose digital IPs

  // RED, AMBER, SOUNDER, HEALTHY
  // RED - POSTIVIE SIGNAL = ON
  // AMBER - POSITIVE SIGNAL = ON
  // HEALTHy - HEALTHY SIGNAL TO BE HEALTHY
  // SOUNDER - ODD ONE OUT, don't get a sounder input, then not sounding

  // Need to hold what was in last cycle if wanting to properly unlatch from alarms etc
  uint8_t digiIn1 = PE11; // External Fault Input, Used in place of legacy IP3 fault, 24v required here for no fault
  uint8_t digiIn2 = PE10; // Incoming RED signal from upstream system
  uint8_t digiIn3 = PE9;  // Incoming AMBER signal from upstream system
  uint8_t digiIn4 = PE8;  // Incoming SOUNDER signal from upstream system
  uint8_t digiIn5 = PE7;  // Incoming HEALTHY signal from upstream system

  bool digiIn1_LastCycle;
  bool digiIn2_LastCycle;
  bool digiIn3_LastCycle;
  bool digiIn4_LastCycle;
  bool digiIn5_LastCycle;
};

struct __attribute__((packed)) TransmitData {
  uint8_t preHeader;              // 1 Byte
  uint8_t header;                 // 1 Byte
  uint16_t SensorRawValues[8];    // 16 bytes
  int16_t SensorScaledValues[8];  // 16 bytes
  uint8_t SensorAlarmsRed;        // 4 bytes
  uint8_t SensorAlarmsAmb;        // 4 bytes
  uint8_t SystemOutputStates;       // 4 bytes
  uint8_t DigiInputStatus;        // 4 bytes
  uint8_t OutputStatus;        // 4 bytes
  uint8_t SystemStatBools;        // 4 bytes
  uint8_t SystemFaultBools;       // 4 bytes
  uint8_t CommandResponse;       // 4 bytes
  uint16_t checkSum;               // 1 Byte
  uint16_t crc;                    // NEW — real CRC16 over [header .. checkSum), catches payload corruption
};                                // Total: 44 bytes

struct __attribute__((packed)) SettingsData {  // to send to STM (if too much data split into sensor settings and system settings)
  uint8_t header = 6;                                                                // 1 Byte
  uint8_t command = 0;
  int16_t sensorZeroOffset[8] = { -170, -170, -170, -170, -170, -170, -170, -170 };  // 16 bytes
  int16_t sensorAlarmRedDepSP[8] = { 180, 180, 180, 180, 180, 180, 180, 180 };       // 16 bytes
  int16_t sensorAlarmRedEnrSP[8] = { 230, 230, 230, 230, 230, 230, 230, 230 };       // 16 bytes
  int16_t sensorAlarmAmbDepSP[8] = { 195, 195, 195, 195, 195, 195, 195, 195 };       // 16 bytes
  int16_t sensorAlarmAmbEnrSP[8] = { 220, 220, 220, 220, 220, 220, 220, 220 };       // 16 bytes
  uint16_t sensorScaleMax[8] = { 250, 250, 250, 250, 250, 250, 250, 250 };           // 16 bytes
  uint8_t sensorActive[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };                              // 8 bytes, remember int not bools, due to legacy LOGO limitation
  uint8_t sensorZone[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };                                // Zone offset
  uint16_t systemRedAlarmDelay = 5;                                                  // 2 bytes
  uint16_t systemAmbAlarmDelay = 1;                                                  // 2 bytes
  uint16_t systemInitDelay = 1000;                                                   // 2 bytes, milliseconds
  uint8_t checkSum = 6;
  uint16_t crc = 0;                 // NEW — real CRC16 over the settings payload
};

struct __attribute__((packed)) RecievedSettingsData {  // to send to STM (if too much data split into sensor settings and system settings)
  // uint8_t preHeader;
  uint8_t header;
  uint8_t command = 0;
  int16_t sensorZeroOffset[8]; // 16 bytes
  int16_t sensorAlarmRedDepSP[8];       // 16 bytes
  int16_t sensorAlarmRedEnrSP[8];       // 16 bytes
  int16_t sensorAlarmAmbDepSP[8];       // 16 bytes
  int16_t sensorAlarmAmbEnrSP[8];       // 16 bytes
  uint16_t sensorScaleMax[8];           // 16 bytes
  uint8_t sensorActive[8];                              // 8 bytes, remember int not bools, due to legacy LOGO limitation
  uint8_t sensorZone[8];                                // Zone offset
  uint16_t systemRedAlarmDelay;                                                  // 2 bytes
  uint16_t systemAmbAlarmDelay;                                                  // 2 bytes
  uint16_t systemInitDelay;                                                   // 2 bytes
  uint8_t checkSum;                                                              // 1 Byte
  uint16_t crc;                     // NEW — must be added in the same position as the ESP32 struct
};

struct __attribute__((packed)) IncomingData {
  //uint8_t preHeader; // Don't include this here even though it is sent, since the preheader is removed from the Serial Stream when finding the header
  uint8_t header;                 // 1 Byte
  uint16_t sensorRawValues[8];    // 16 bytes
  int16_t sensorScaledValues[8];  // 16 bytes
  uint8_t sensorAlarms[4];        // 4 bytes
  uint8_t systemStatBools[2];     // 2 bytes
  uint8_t CommandResponse;        // 1 Byte in bools
  uint16_t checkSum;               // 2 Byte 
  uint8_t tempPadding;            // 1 Byte
};

extern RecievedSettingsData recievedSettingsData;
extern IncomingData incomingData;
extern TransmitData transmitData;  // Extern declaration
extern SettingsData settingsData;  // Extern declaration

#endif