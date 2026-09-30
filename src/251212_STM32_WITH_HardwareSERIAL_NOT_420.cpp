// Generic 407VETx
#include <Arduino.h>
#include "Arduino.h"
#include <SGD.h>
#include <TimerSGD.h>
#include "stm32f4xx_hal.h"
#include "hal_conf_extra.h"
#include "TestingCode.cpp"

IWDG_HandleTypeDef hiwdg;

// Flash sector 7 (0x08060000, 128KB) used for settings persistence
#define SETTINGS_FLASH_SECTOR FLASH_SECTOR_7
#define SETTINGS_FLASH_ADDR 0x08060000UL
#define SETTINGS_MAGIC 0xA5B6C7D8UL
#define SETTINGS_STABLE_COUNT 3
#define SENSOR_COUNT 8

static RecievedSettingsData settingsCandidate;
static uint8_t              settingsCandidateStreak = 0;

// Fake "Memory" for testing
bool ActiveMemory[8] = {true, false, false, false, false, false, false, false};
unsigned long REDalarmStartTime[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long AMBalarmStartTime[8] = {0, 0, 0, 0, 0, 0, 0, 0};

bool RED_alarmTriggered[8] = {false, false, false, false, false, false, false, false};          // After the delay when a sensor falls out of range this changes to true
bool RED_alarmTriggered_LastLoop[8] = {false, false, false, false, false, false, false, false}; // After the delay when a sensor falls out of range this changes to true
bool AMB_alarmTriggered[8] = {false, false, false, false, false, false, false, false};          // After the delay when a sensor falls out of range this changes to true

unsigned long millisLastGoodData = 0;  // place to store start time of requestDataTimer
unsigned long millisSinceLastGoodData; // elapsed time since we last received good data from ESP

const int noOfSamples = 10;                                 // Number of readings to average (samples), for smoothing
int SampleNo = 0;                                           // Current sample number: Pointer in the array
int SensorRawValuesArray[SENSOR_COUNT][noOfSamples];        // Used to store samples of Raw DAC readings to average (smooth)
int SensorRawValues_LastLoop[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Used for A2D Fault. Used to store the current Raw to use as a comparison next cycle

bool Active[8] = {false, false, false, false, false, false, false, false};
const int sensorPins[8] = {PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA0};

uint8_t AnalogueOutput = PA9;

HardwareSerial ESPSerial(PC11, PC10); // Rx, TX pins from/to ESP32
// HardwareSerial SoftDbg(PA10, PA9); // RX, TX XXX ADDEDFORSERIALTEST (PA10 not used)

GasDetection gasDetection;
Custom_Timer timer;

bool underRangeActive = false;          // Tracks if the timer is running
bool A2D_TimerActive = false;           // Tracks if the timer is running
bool ExternalFault_TimerActive = false; // Tracks if the timer is running
unsigned long sendDataStartTm = 0;      // place to store start time of requestDataTimer
bool underRangeDetected = false;
static unsigned long lastParamMemCheckTm = 0;
const unsigned long ParamMemCheckInterval = 60000; // well inside the 24h requirement

static unsigned long lastLedToggleTm = 0;
const unsigned long LedToggleMinInterval = 250; // caps LED_PIN_1 to a visible heartbeat rate, no matter how fast valid packets arrive

int checkSums[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int currnentChecksum = 0;
static uint8_t commFailCounter = 0; // counts consecutive bad returns

static RecievedSettingsData storedSettings;
static bool storedSettingsValid = false;
static bool startupCheckDone = false;
static uint8_t settingsMismatchCount = 0;
static bool settingsFaultActive = false;


static uint16_t crc16(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

static bool flashSaveSettings(const RecievedSettingsData *data)
{
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef eraseInit = {};
  eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseInit.Sector = SETTINGS_FLASH_SECTOR;
  eraseInit.NbSectors = 1;
  eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  uint32_t sectorError = 0;
  if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return false;
  }
  HAL_IWDG_Refresh(&hiwdg); // sector erase can take up to ~2s on its own, close to the IWDG window — feed it right after the slow part completes
  uint32_t magic = SETTINGS_MAGIC;
  for (int i = 0; i < 4; i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, SETTINGS_FLASH_ADDR + i,
                          ((uint8_t *)&magic)[i]) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return false;
    }
  }
  const uint8_t *src = (const uint8_t *)data;
  for (size_t i = 0; i < sizeof(RecievedSettingsData); i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, SETTINGS_FLASH_ADDR + 4 + i,
                          src[i]) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return false;
    }
  }

  uint16_t crc = crc16(src, sizeof(RecievedSettingsData));
  uint32_t crcAddr = SETTINGS_FLASH_ADDR + 4 + sizeof(RecievedSettingsData);
  for (int i = 0; i < 2; i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, crcAddr + i,
                          ((uint8_t *)&crc)[i]) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return false;
    }
  }

  HAL_FLASH_Lock();
  return true;
}

static bool flashLoadSettings(RecievedSettingsData *data)
{
  uint32_t magic = *(volatile uint32_t *)SETTINGS_FLASH_ADDR;
  if (magic != SETTINGS_MAGIC)
    return false;
  const uint8_t *src = (const uint8_t *)(SETTINGS_FLASH_ADDR + 4);
  memcpy(data, src, sizeof(RecievedSettingsData));
  return true;
}

// Compare only the settings payload — skips header[0], command[1], and checkSum[last byte]
static bool settingsMatch(const RecievedSettingsData *a, const RecievedSettingsData *b)
{
  const uint8_t *pa = (const uint8_t *)a;
  const uint8_t *pb = (const uint8_t *)b;
  return memcmp(pa + 2, pb + 2, sizeof(RecievedSettingsData) - 3) == 0;
}

void sendData()
{

  // Generate or update checksum
  if (currnentChecksum == 0)
  {
    // start with a new random seed
    transmitData.checkSum = rand() % 101;
  }
  else
  {
    // increment based on previous
    transmitData.checkSum = checkSums[currnentChecksum - 1] + 1;
  }

  // store the checksum
  checkSums[currnentChecksum] = transmitData.checkSum;

  // advance index and wrap at 10
  currnentChecksum = (currnentChecksum + 1) % 10;
  // ESPSerial.flush();

  // NEW: compute a real CRC over the payload, starting at `header` (skip preHeader,
  // which never reaches the parsed struct on the receiving end) and stopping
  // just before `checkSum` (the CRC field itself obviously can't cover itself).
  transmitData.crc = crc16(((uint8_t *)&transmitData) + offsetof(TransmitData, header),
                           offsetof(TransmitData, checkSum) - offsetof(TransmitData, header));

  uint8_t buffer[sizeof(transmitData)] = {0}; // Zero-initialize the buffer
  // digitalWrite(gasDetection.LED_PIN_1, !digitalRead(gasDetection.LED_PIN_1));

  // Copy struct data into the buffer
  memcpy(buffer, &transmitData, sizeof(transmitData));

  // Send data over Serial in one go
  size_t available = ESPSerial.availableForWrite();
  if (available > sizeof(buffer))
  {
    ESPSerial.write(buffer, sizeof(buffer));
  }
};

static void checkAndCommitSettings(const RecievedSettingsData* received) {
  if (!storedSettingsValid) {
    // First ever boot — nothing to compare against yet, accept this packet as the baseline.
    flashSaveSettings(received);
    memcpy(&storedSettings, received, sizeof(storedSettings));
    storedSettingsValid = true;
    settingsCandidateStreak = 0;
    return;
  }

  if (settingsMatch(received, &storedSettings)) {
    // Matches what's already saved — nothing to do.
    settingsCandidateStreak = 0;
    return;
  }

  // Differs from the saved baseline — is it the same new value as last time, or something new again?
  if (settingsCandidateStreak > 0 && settingsMatch(received, &settingsCandidate)) {
    settingsCandidateStreak++;
  } else {
    memcpy(&settingsCandidate, received, sizeof(settingsCandidate));
    settingsCandidateStreak = 1;
  }

  if (settingsCandidateStreak >= SETTINGS_STABLE_COUNT) {
    // Stable for N consecutive packets — commit it as the new baseline.
    flashSaveSettings(&settingsCandidate);
    memcpy(&storedSettings, &settingsCandidate, sizeof(storedSettings));
    settingsCandidateStreak = 0;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup Core for the loop
void setup()
{
  // This sets up an independent hardware timer (IWDG)
  //  that counts down on its own internal clock, separate
  //  from your main program; if HAL_IWDG_Refresh() isn't called regularly
  //  to reset that countdown, the timer expires and forces the STM32 to reboot.
  //  It's a safety net that catches your firmware hanging or locking up.
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32; // adjust for your desired timeout
  hiwdg.Init.Reload = 4095;                 // max reload for this prescaler ≈ 4s timeout on 32kHz LSI
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
  }

  gasDetection.setupPinModes();

  ESPSerial.begin(115200); // UART from/to ESP32
  ESPSerial.setTimeout(10);

  // SoftDbg.begin(9600);  // XXX ADDEDFORSERIALTEST
  // SoftDbg.println("Soft serial debug"); // XXX ADDEDFORSERIALTEST

  timer.InitialiseStartTime = timer.StartTimer();
  storedSettingsValid = flashLoadSettings(&storedSettings);

  // Initialise the Samples to zero
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    for (int j = 0; j < noOfSamples; j++)
    {
      SensorRawValuesArray[SENSOR_COUNT][noOfSamples] = 0;
    }
  }

  analogWriteResolution(10);                    // Set Analogue Output Resolution to 10 Bit, 0-1023
  analogWrite(gasDetection.AnalogueOutput, 0);  // Initialise the Analogue Output to zero as good practice

} // End of Setup

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  HAL_IWDG_Refresh(&hiwdg); // Refresh the watchdog timer to prevent reset

  // Sets Outputs for Red, Amber, Sounder ON / Healthy and Green OFF as a FAILSAFE STARTING POINT
  gasDetection.setFailsafeOutputs();

  digitalWrite(gasDetection.Fail_Safe_Relay, HIGH); // Turn on the Fail Safe Relay

  transmitData.preHeader = 8; //  an arbitary value as a trigger
  transmitData.header = 9;    //  an arbitary value as a trigger

  if (millis() - lastParamMemCheckTm >= ParamMemCheckInterval)
  {
    lastParamMemCheckTm = millis();
    RecievedSettingsData scratch;
    settingsFaultActive = !flashLoadSettings(&scratch);
  }

  // GET SENSOR READINGS
  // ANALOGUE SMOOTHING SOFTWARE HERE:
  for (int i = 0; i < SENSOR_COUNT; i++)
  { // loop through the 8 sensors

    SensorRawValuesArray[i][SampleNo] = gasDetection.GetSensorReading(sensorPins[i]); // get the current raw value, put it in the sampling array.

    // get the average of the array contents for the current sensor [i]
    int readingTot = 0; // a place to hold the total of the samples
    for (int j = 0; j < noOfSamples; j++)
    {                                           // loop through the samples
      readingTot += SensorRawValuesArray[i][j]; // Get the total value of the stored array
    }
    float readingAvg = (float)readingTot / noOfSamples; // FIX: float division now, fractional part preserved
    int readingAvgRounded = round(readingAvg);          // now this rounding actually does something
    transmitData.SensorRawValues[i] = readingAvgRounded;
  }
  SampleNo++; // increment the log counter for the next sample (next program cycle)
  if (SampleNo >= noOfSamples)
    SampleNo = 0; // reset the sample number to start again if neccessary
  // ANALOGUE SMOOTHING SOFTWARE END

  if (gasDetection.SystemStatBoolsArr[0] == 1)
  { // if we are initialised
    // SCALE SENSOR READINGS
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      int SensorRawValueOffset = transmitData.SensorRawValues[i] + settingsData.sensorZeroOffset[i] + 170;                                            // apply the zero offset, using the traditional -170 value
      transmitData.SensorScaledValues[i] = map(SensorRawValueOffset, gasDetection.ADC_4mA, gasDetection.ADC_20mA, 0, settingsData.sensorScaleMax[i]); // map(value, fromLow, fromHigh, toLow, toHigh)

      // CHECK EACH ACTIVE SENSOR - IS IT IN RED? AMBER? (USE INDIVIDUAL SENSOR TIMERS)
      if (settingsData.sensorActive[i] == true)
      {
        // Get the alarm status (replace with actual threshold comparison)
        bool isInRedAlarm = gasDetection.getAlarmStatus(transmitData.SensorScaledValues[i], settingsData.sensorAlarmRedEnrSP[i], settingsData.sensorAlarmRedDepSP[i]);
        bool isIAmberAlarm = gasDetection.getAlarmStatus(transmitData.SensorScaledValues[i], settingsData.sensorAlarmAmbEnrSP[i], settingsData.sensorAlarmAmbDepSP[i]);

        // RED - If sensor is in alarm, check if the 5-second countdown is needed
        if (isInRedAlarm)
        {
          // If alarm wasn't triggered yet, start the countdown
          if (!RED_alarmTriggered[i])
          {
            if (REDalarmStartTime[i] == 0)
            {
              REDalarmStartTime[i] = timer.StartTimer();
            }
            RED_alarmTriggered[i] = timer.TimerElapsed(timer.systemRedAlarmDelay, REDalarmStartTime[i]);
          }
        }
        else
        {
          // If the sensor is not in alarm, reset the countdown and alarm status
          RED_alarmTriggered[i] = false;
          REDalarmStartTime[i] = 0; // Reset the timer
        }

        // AMBER
        if (isIAmberAlarm)
        {
          // If alarm wasn't triggered yet, start the countdown
          if (!AMB_alarmTriggered[i])
          {
            if (AMBalarmStartTime[i] == 0)
            {
              AMBalarmStartTime[i] = timer.StartTimer(); // Start the timer
            }

            AMB_alarmTriggered[i] = timer.TimerElapsed(timer.systemAmbAlarmDelay, AMBalarmStartTime[i]);
          }
        }
        else
        {
          // If the sensor is not in alarm, reset the countdown and alarm status
          AMB_alarmTriggered[i] = false;
          AMBalarmStartTime[i] = 0; // Reset the timer
        }
      }
      else
      { // SENSOR IS NOT ACTIVE
        RED_alarmTriggered[i] = false;
        REDalarmStartTime[i] = 0; // Reset the timer

        AMB_alarmTriggered[i] = false;
        AMBalarmStartTime[i] = 0; // Reset the timer
      }
    }

    // WE HAVE THE SENSOR STATUS
    // NOW LOOK AT THE SYSTEM STATUS:

    ////////////////////////// Under Range detection section
    underRangeDetected = false;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      if (settingsData.sensorActive[i] == true)
      {
        if (transmitData.SensorRawValues[i] <= (gasDetection.ADC_4mA / 2))
        { // Check to see if Raw value is less than 2ma
          underRangeDetected = true;
        }
      }
    }

    if (underRangeDetected)
    { // if we are in the underrange state
      if (!underRangeActive)
      { // And if the timer isn't running
        underRangeActive = true;
        timer.UnderRangeStartTime = timer.StartTimer(); //  Start the timer
      }
    }
    else
    { // If not in underange state
      if (underRangeActive)
      { // and if the timer has been activated
        underRangeActive = false;
        timer.UnderRangeStartTime = 0; // Reset timer
      }
    }

    gasDetection.SystemFaultBoolsArr[0] = timer.TimerElapsed(timer.UnderRangeDelay, timer.UnderRangeStartTime); // Check to see if timer has elsapsed, return true if it has.

    ////////////////////////// A2D Fault detection section
    bool readingsChanged = false;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      if (SensorRawValues_LastLoop[i] != transmitData.SensorRawValues[i])
      { // if the sensor readings are different to what they were in the last loop
        readingsChanged = true;
      }
    }

    if (!readingsChanged)
    { // if readings haven't changed
      if (!A2D_TimerActive)
      { // and if the A2D timer isn't active
        A2D_TimerActive = true;
        timer.A2D_StartTime = timer.StartTimer(); // Start the timer
      }
    }
    else
    { // If readings have changed
      if (A2D_TimerActive)
      { // and timer is still active
        A2D_TimerActive = false;
        timer.A2D_StartTime = 0; // Reset timer
      }
    }

    gasDetection.SystemFaultBoolsArr[1] = timer.TimerElapsed(timer.A2D_Delay, timer.A2D_StartTime); // Check to see if timer has elsapsed, return true if it has.

    ////////////////////////// Coms to Slave Fault detection section

    if (millisSinceLastGoodData >= 5000 || commFailCounter >= 5) {
      gasDetection.SystemFaultBoolsArr[2] = 1;
    }
    else
    {
      gasDetection.SystemFaultBoolsArr[2] = 0;
    }

    ////////////////////////// External Fault detection section

    bool ExternalInput1 = false;
    ExternalInput1 = !digitalRead(gasDetection.digiIn1);

    if (!ExternalInput1)
    { // if we don't have the external input active
      if (!ExternalFault_TimerActive)
      { // and if the external fault timer isn't active
        ExternalFault_TimerActive = true;
        timer.ExternalStartTime = timer.StartTimer(); // Start the timer
      }
    }
    else
    { // If we do get an external input active
      if (ExternalFault_TimerActive)
      { // and timer is still active
        ExternalFault_TimerActive = false;
        timer.ExternalStartTime = 0; // Reset timer
      }
    }

    gasDetection.SystemFaultBoolsArr[3] = timer.TimerElapsed(timer.ExternalFaultDelay, timer.ExternalStartTime); // Check to see if timer has elapsed, return true if it has.

    gasDetection.SystemFaultBoolsArr[4] = 0;

    // Output fault monitoring - Monitoring INPUT should be the OPPOSITE of the OUTPUTS
    if (digitalRead(gasDetection.RedOutputMonitorIP) == digitalRead(gasDetection.RedOutput))
    {
      gasDetection.SystemFaultBoolsArr[4] = 1;
    }
    if (digitalRead(gasDetection.AmbOutputMonitorIP) == digitalRead(gasDetection.AmberOutput))
    {
      gasDetection.SystemFaultBoolsArr[4] = 1;
    }
    if (digitalRead(gasDetection.GreenOutputMonitorIP) == digitalRead(gasDetection.GreenOutput))
    {
      gasDetection.SystemFaultBoolsArr[4] = 1;
    }
    if (digitalRead(gasDetection.SounderOutputMonitorIP) == digitalRead(gasDetection.SounderOutput))
    {
      gasDetection.SystemFaultBoolsArr[4] = 1;
    }
    if (digitalRead(gasDetection.HealthyOutputMonitorIP) == digitalRead(gasDetection.HealthyOutput))
    {
      gasDetection.SystemFaultBoolsArr[4] = 1;
    }

    int test1 = bitRead(recievedSettingsData.command, 6);
    bool test2 = bitRead(recievedSettingsData.command, 6);
    if (bitRead(recievedSettingsData.command, 6) == 1)
    {
      gasDetection.SystemFaultBoolsArr[5] = true;
    }
    else
    {
      gasDetection.SystemFaultBoolsArr[5] = false;
    }
    // = bitRead(recievedSettingsData.command, 5);

    // settingsFaultActive = true;  // XXX TEST ONLY - forces settings fault, remove this line
    gasDetection.SystemFaultBoolsArr[6] = settingsFaultActive;

    transmitData.SystemFaultBools = gasDetection.bools2Int(gasDetection.SystemFaultBoolsArr); // Convert the Fault Status Bools to one int ready for sending

    // END OF FAULT SOFTWARE (HEALTHY IS DONE LATER...)
    ///////////////////////////////////////////////////

    ///////////////////////////////////////////////////
    // Now determine all the System Statuses

    // OVER RANGE (this is a state, not a fault)
    bool OverRangeActive = false;
    // No timer needed for this. Will return 1 if ANY are over threshold - Will not trigger fault condtion on update
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      if (settingsData.sensorActive[i])
      {
        if (transmitData.SensorRawValues[i] > (gasDetection.ADC_20mA * 1.1))
        { // works on RAW input, not scaled input
          OverRangeActive = true;
        }
      }
    }

    gasDetection.SystemStatBoolsArr[6] = OverRangeActive;

    // convert bools to ints
    transmitData.SensorAlarmsRed = gasDetection.bools2Int(RED_alarmTriggered); // perform the conversion
    transmitData.SensorAlarmsAmb = gasDetection.bools2Int(AMB_alarmTriggered); // perform the conversion

    // SYSTEM RED ALARM
    // First for the local system status from the onboard sensors, and set the System Status to match (for Modbus)
    bool RedLocal = false;
    if (transmitData.SensorAlarmsRed || gasDetection.SystemStatusRedLatch)
    {                                         // System Red status
      gasDetection.SystemStatBoolsArr[1] = 1; // Red Bit in the Status Array
      gasDetection.SystemStatusRedLatch = 1;  // Set the latch red alarm on
      RedLocal = true;
    }
    else
    {
      gasDetection.SystemStatBoolsArr[1] = 0; // Red Bit in the Status Array
      RedLocal = false;
    }

    // Second: Now apply the Digital Input Status to influence the Physical Outputs
    bool RedDigiInput = digitalRead(gasDetection.digiIn2);
    bool RedCommsInput = bitRead(recievedSettingsData.command, 2);
    bool RedPhysicalOutput; // Used to determine the state of the Green Output Later
    if (RedDigiInput || RedLocal || RedCommsInput)
    {
      digitalWrite(gasDetection.RedOutput, 1); // Turn on physical red output
      RedPhysicalOutput = 1;
    }
    else
    {
      digitalWrite(gasDetection.RedOutput, 0); // Turn off physical red output
      RedPhysicalOutput = 0;
    }

    // SYSTEM AMBER ALARM
    // First for the local system status from the onboard sensors, and set the System Status to match (for Modbus)
    bool AmberLocal = false;
    if (transmitData.SensorAlarmsAmb || transmitData.SystemFaultBools > 0)
    {                                         // System Amber status. System fault bools is an int
      gasDetection.SystemStatBoolsArr[2] = 1; // Amber Bit in the Status Array
      AmberLocal = true;
    }
    else
    {
      gasDetection.SystemStatBoolsArr[2] = 0; // Amber Bit in the Status Array
      AmberLocal = false;
    }

    // Second: Now apply the Digital Input Status to influence the Physical Outputs
    bool AmbDigiInput = digitalRead(gasDetection.digiIn3);
    bool AmberCommsInput = bitRead(recievedSettingsData.command, 3);
    bool AmberPhysicalOutput; // Used to determine the state of the Green Output Later
    if (AmbDigiInput || AmberLocal || AmberCommsInput)
    {
      digitalWrite(gasDetection.AmberOutput, 1); // Turn on physical amber output
      AmberPhysicalOutput = 1;
    }
    else
    {
      digitalWrite(gasDetection.AmberOutput, 0); // Turn off physical amber output
      AmberPhysicalOutput = 0;
    }

    // UNLATCH MUTE
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      if (RED_alarmTriggered_LastLoop[i] == 0 && RED_alarmTriggered[i] == 1)
      {                                // if the sensor wasn't in red alarm cycle but is this cycle then we have a new red alarm.
        gasDetection.SounderMuted = 0; // Unlatch the mute function.
      }
    }

    // SOUNDER
    // First for the local system status from the onboard sensors, and set the System Status to match (for Modbus)
    bool SounderLocal = false;
    if (gasDetection.SystemStatusRedLatch && !gasDetection.SounderMuted)
    {
      gasDetection.SystemStatBoolsArr[4] = 1; // Sounder Active bit in the array
      SounderLocal = true;
    }
    else
    {
      gasDetection.SystemStatBoolsArr[4] = 0; // Sounder Active bit in the array
      SounderLocal = false;
    }

    // Second: Now apply the Digital Input Status to influence the Physical Outputs
    bool SounderDigiInput = !digitalRead(gasDetection.digiIn4);
    bool SounderCommsInput = bitRead(recievedSettingsData.command, 4);
    int SounderResult = 0;
    if (SounderDigiInput || SounderLocal || SounderCommsInput)
    {
      digitalWrite(gasDetection.SounderOutput, 1); // Turn on physical sounder output
      SounderResult = 1;
    }
    else
    {
      digitalWrite(gasDetection.SounderOutput, 0); // Turn off physical sounder output
      SounderResult = 0;
    }

    // HEALTHY
    // First for the local system status from the onboard sensors, and set the System Status to match (for Modbus)
    bool HealthyLocal = false;
    if (transmitData.SystemFaultBools > 0)
    {                                         // We have a fault
      gasDetection.SystemStatBoolsArr[5] = 0; // Show we are not healthy to the ESP
      HealthyLocal = false;
    }
    else
    {
      gasDetection.SystemStatBoolsArr[5] = 1;
      HealthyLocal = true;
    }

    // Second: Now apply the Digital Input Status to influence the Physical Outputs
    bool HealthyDigiInput = !digitalRead(gasDetection.digiIn5);
    bool HealthyCommsInput = bitRead(recievedSettingsData.command, 5);
    if (HealthyDigiInput && HealthyLocal && HealthyCommsInput)
    {
      digitalWrite(gasDetection.HealthyOutput, 1); // Turn on physical healthy output
    }
    else
    {
      digitalWrite(gasDetection.HealthyOutput, 0); // Turn off physical healthy output
    }

    // SYSTEM GREEN
    // First for the local system status from the onboard sensors, and set the System Status to match (for Modbus)
    if (!transmitData.SensorAlarmsAmb && !transmitData.SensorAlarmsRed && !gasDetection.SystemStatusRedLatch && transmitData.SystemFaultBools == 0)
    { // System Green status if not Red and not Amber
      gasDetection.SystemStatBoolsArr[3] = 1;
    }
    else
    {
      gasDetection.SystemStatBoolsArr[3] = 0;
    }
    // Second: Now apply the Digital Input Status to influence the Physical Outputs
    if (RedPhysicalOutput || AmberPhysicalOutput)
    {                                            // If Physical outputs are Red or Amber **MAYBE CHANGE THIS TO DIGITALREAD XXXX TO REFERNCE THE ACTUAL OUTPUT STATES RATHER THAN USING LOCAL BOOL VARIABLES?
      digitalWrite(gasDetection.GreenOutput, 0); // Turn off Green
    }
    else
    {
      digitalWrite(gasDetection.GreenOutput, 1); // Else turn on Green
    }

    // DIGI INPUTS GO HERE
    gasDetection.SystemDigiInputs[0] = !digitalRead(gasDetection.digiIn1);
    gasDetection.SystemDigiInputs[1] = !digitalRead(gasDetection.digiIn2);
    gasDetection.SystemDigiInputs[2] = !digitalRead(gasDetection.digiIn3);
    gasDetection.SystemDigiInputs[3] = !digitalRead(gasDetection.digiIn4);
    gasDetection.SystemDigiInputs[4] = !digitalRead(gasDetection.digiIn5);
    gasDetection.SystemDigiInputs[5] = 0;
    gasDetection.SystemDigiInputs[6] = 0;
    gasDetection.SystemDigiInputs[7] = 0;

    transmitData.DigiInputStatus = gasDetection.bools2Int(gasDetection.SystemDigiInputs);

    // OUTPUTS GO HERE
    gasDetection.SystemOutputs[0] = digitalRead(gasDetection.RedOutput);
    gasDetection.SystemOutputs[1] = digitalRead(gasDetection.AmberOutput);
    gasDetection.SystemOutputs[2] = digitalRead(gasDetection.GreenOutput);
    gasDetection.SystemOutputs[3] = SounderResult; // ************REPEAT THIS, UPDATE THE OTHERS!!!!!!!!!!!!!!
    gasDetection.SystemOutputs[4] = digitalRead(gasDetection.HealthyOutput);
    gasDetection.SystemOutputs[5] = 0;
    gasDetection.SystemOutputs[6] = 0;
    gasDetection.SystemOutputs[7] = 0;

    transmitData.OutputStatus = gasDetection.bools2Int(gasDetection.SystemOutputs);

    ///////
    // NOW WE HAVE DETERMINED ALL THE SYSTEM STATUS BITS, GET SYSTEM STATUS DATA READY FOR SENDING TO ESP AS AN INT (FOR DATA TX EFFICIENCY)
    transmitData.SystemStatBools = gasDetection.bools2Int(gasDetection.SystemStatBoolsArr); // perform the conversion of bools array to an int

    // NOTE THE RED STATUS OF EACH SENSOR SO WE CAN DETECT A NEW RED STATUS ON THE NEXT SCAN CYCLE
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      RED_alarmTriggered_LastLoop[i] = RED_alarmTriggered[i];
      SensorRawValues_LastLoop[i] = transmitData.SensorRawValues[i];
    }
  }

  //////////////////
  // ANALOGUE OUTPUT
  //  output nothing to begin with as a safety precaution
  //  The lower the output value, the less safe the state i.e. to show Green at the beacon requires a high signal of ~20mA, anything below 10 shows Red
  analogWrite(gasDetection.AnalogueOutput, 0);

  /* mA Levels:
  20 Only Green
  14 Only Amber
  10 Red active but no sounder
  4 Red Active plus Sounder
  */
  int PWMvalue = 0;       // Value to set the PWM output level, initialise to zero for safety
  int PWMvaluePerMa = 50; // PWM bit value per mA
  if (!digitalRead(gasDetection.AmberOutput) && !digitalRead(gasDetection.RedOutput))
    PWMvalue = PWMvaluePerMa * 20; // Green
  if (digitalRead(gasDetection.AmberOutput))
    PWMvalue = PWMvaluePerMa * 14; //  amber
  if (digitalRead(gasDetection.RedOutput) && !digitalRead(gasDetection.SounderOutput))
    PWMvalue = PWMvaluePerMa * 10; // Red No Sounder
  if (digitalRead(gasDetection.RedOutput) && digitalRead(gasDetection.SounderOutput))
    PWMvalue = PWMvaluePerMa * 4; // Red with Sounder

 analogWrite(gasDetection.AnalogueOutput, PWMvalue);  // Send the value to the analogue output

  // transmitData.checkSum = PWMvalue; // TEMPORARY PLACE TO TRANSMIT TO ESP FOR DEBUGGING

  ///////////////////////////////////////////////////////////////////////////////////////
  // RECIEVING DATA STUFF

  char expectedPreHeader = 8;
  // char expectedheader = 6;  // 1 Byte

  char wasteBinBuffer[256];                                // Somewhere to dump unwanted data; sized to safely fit readBytesUntil's scan length below
  millisSinceLastGoodData = millis() - millisLastGoodData; // recalculate time since we last received good data from ESP

  // Find the start of the incoming Serial Stream
  if (ESPSerial.available())
  {
    // ESPSerial.setTimeout(1000);

    if (ESPSerial.available() >= (int)sizeof(RecievedSettingsData) + 1)
    {
      // if there is a full set of Data to read (plus 1 allows for the preheader which is not part of the receiving struct)
      // if there is data from the ESP chip
      ESPSerial.readBytesUntil(expectedPreHeader, wasteBinBuffer, sizeof(wasteBinBuffer)); // read and remove bytes from the incoming stream until we find (and remove) the pre-header value

      char headerCheckChar = ESPSerial.peek();
      /// CHANGE BACK TO 6
      if (headerCheckChar == 6)
      {
        ESPSerial.readBytes((char *)&recievedSettingsData, sizeof(RecievedSettingsData));

        // Verify payload integrity before trusting anything in this packet
        uint16_t expectedCrc = crc16((uint8_t *)&recievedSettingsData, offsetof(RecievedSettingsData, checkSum));
        bool crcOk = (recievedSettingsData.crc == expectedCrc);

        // Checksum echo validation (confirms ESP32 is alive and recently heard from us)
        bool found = false;
        for (int i = 0; i < 10; i++)
        {
          if (recievedSettingsData.checkSum == checkSums[i])
          {
            found = true;
            break;
          }
        }

        // Treat this packet as failed if EITHER check fails — but only count it once,
        // so one bad packet is one failure, not two.
        if (found && crcOk)
        {
          commFailCounter = 0; // fully good packet, reset failures
        }
        else
        {
          if (commFailCounter < 255)
            commFailCounter++; // capped so it can't wrap around during a long outage
        }

        // Only apply the received settings if the payload itself checked out — a failed CRC
        // means this packet's content can't be trusted, regardless of what the echo check says.
        if (crcOk)
        {
          memcpy(&settingsData, &recievedSettingsData, sizeof(recievedSettingsData));
          checkAndCommitSettings(&recievedSettingsData);   // NEW
        }

        // if we still have data available, there was an excess of data, get rid of it
        while (ESPSerial.available() > 0)
        {
          ESPSerial.read();
        }
        if (found && crcOk && millis() - lastLedToggleTm >= LedToggleMinInterval)
        {
          lastLedToggleTm = millis();
          digitalWrite(gasDetection.LED_PIN_1, !digitalRead(gasDetection.LED_PIN_1)); // heartbeat, not a raw packet-rate strobe
        }

        millisLastGoodData = millis(); // Reset counter — tracks "a packet arrived", separate from "it was trustworthy"

        // Startup settings verification: compare received settings against flash-stored baseline.
        // SystemFaultBoolsArr[6] is set after 3 consecutive mismatches on boot.
        // After startup, flash is kept current so the next boot compares to the latest known settings.
        // if (!startupCheckDone) {
        //   if (!storedSettingsValid) {
        //     // First ever boot — no stored settings yet; save this packet as the baseline
        //     flashSaveSettings(&recievedSettingsData);
        //     memcpy(&storedSettings, &recievedSettingsData, sizeof(storedSettings));
        //     storedSettingsValid = true;
        //     startupCheckDone    = true;
        //   } else if (settingsMatch(&recievedSettingsData, &storedSettings)) {
        //     settingsMismatchCount = 0;
        //     startupCheckDone   = true;
        //     settingsFaultActive = false;
        //   } else {
        //     settingsMismatchCount++;
        //     if (settingsMismatchCount >= 3) {
        //       startupCheckDone   = true;
        //       settingsFaultActive = true;
        //       // Update stored so next boot compares to the current (new) settings
        //       flashSaveSettings(&recievedSettingsData);
        //       memcpy(&storedSettings, &recievedSettingsData, sizeof(storedSettings));
        //     }
        //   }
        // } else if (!settingsMatch(&recievedSettingsData, &storedSettings)) {
        //   // After startup: settings legitimately changed — keep flash current for next boot
        //   flashSaveSettings(&recievedSettingsData);
        //   memcpy(&storedSettings, &recievedSettingsData, sizeof(storedSettings));
        // }
      }
    }
  }

  // settingsFaultActive = true;

  // COMMAND HANDLING:

  transmitData.CommandResponse = 0; // First, reset this int ready for fresh data

  // CALL THE ROUTINES FOR MUTE OR RESET IF WE HAVE RECEIVED THE COMMAND FROM ESP:
  if (gasDetection.checkCommandRisingEdge(0))
    gasDetection.MuteAlarm();
  if (gasDetection.checkCommandRisingEdge(1))
    gasDetection.ResetAlarm();
  gasDetection.lastCommand = recievedSettingsData.command;

  // Create a timer to Send Data to the ESP chip every x milliseconds
  unsigned long TmInterval = 500;                     // time interval
  unsigned long elapsed = millis() - sendDataStartTm; // elapsed: duration
  bool sendDataNow = (elapsed >= TmInterval);         // return 1 if elapsed time is greater than the set Timer Interval, else 0

  if (sendDataNow)
  {
    sendDataStartTm = millis(); // Reset the start time
    size_t available = ESPSerial.availableForWrite();

    if (available > sizeof(transmitData))
    {
      sendData(); // SEND DATA TO THE ESP
    }
  }

  // INITIALISATION TIMER
  if (timer.TimerElapsedInitialise(settingsData.systemInitDelay * 10, timer.InitialiseStartTime))
  {                                         // If the initalisation timer has elapsed (systemInitDelay is already in milliseconds)
    gasDetection.SystemStatBoolsArr[0] = 1; // Flag that the system has initialised
  }

  ramSelfTestStep();
  gasDetection.SystemFaultBoolsArr[7] = ramTestFault;

}
