// --- 4.6 f) Volatile memory (RAM) self-test ---
// Dedicated scratch region, not used for any live data — tested each loop() pass
// for stuck-at and adjacent-bit coupling faults. Chunk-based so it costs almost
// nothing per pass; the whole buffer gets swept many times a second in practice,
// far inside the "at least once per 24h" requirement.
#define RAM_TEST_BUFFER_SIZE   256
#define RAM_TEST_CHUNK_SIZE    8       // bytes tested per pass -> full sweep every 32 passes
#include "Arduino.h"  // because serial doesnt work without it in the .cpp file

static volatile uint8_t ramTestBuffer[RAM_TEST_BUFFER_SIZE];
static uint16_t ramTestIndex          = 0;
static bool     ramTestFailedThisSweep = false;
static bool     ramTestFault           = false;  // reported result — only updated once a full sweep completes

static void ramSelfTestStep() {
  bool chunkOk = true;

  for (uint16_t i = 0; i < RAM_TEST_CHUNK_SIZE; i++) {
    uint16_t addr = ramTestIndex + i;

    ramTestBuffer[addr] = 0x00; if (ramTestBuffer[addr] != 0x00) chunkOk = false;
    ramTestBuffer[addr] = 0xFF; if (ramTestBuffer[addr] != 0xFF) chunkOk = false;
    ramTestBuffer[addr] = 0xAA; if (ramTestBuffer[addr] != 0xAA) chunkOk = false;  // 10101010 — catches adjacent-bit coupling
    ramTestBuffer[addr] = 0x55; if (ramTestBuffer[addr] != 0x55) chunkOk = false;  // 01010101 — inverse of above
  }

  if (!chunkOk) ramTestFailedThisSweep = true;

  ramTestIndex += RAM_TEST_CHUNK_SIZE;
  if (ramTestIndex >= RAM_TEST_BUFFER_SIZE) {
    ramTestIndex = 0;
    ramTestFault           = ramTestFailedThisSweep;  // only report once a full sweep has actually completed
    ramTestFailedThisSweep = false;                    // reset for the next sweep
  }
}