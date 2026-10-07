# 261007_V6SafeGuardSTM_Bespoke_mAoutput

Bespoke mA-output variant of the SafeGuard STM32F407VET6 gas detection controller.

## Fork record

| | |
|---|---|
| Parent repo | `rpetho/260629-090729-genericSTM32F407VET6` |
| Forked at commit | `fbc91d6` (2026-09-30) |
| Forked on | 2026-10-07 |

**This is a hard fork.** Fixes do **not** flow between this repo and the parent in
either direction. Anything found in the shared code must be applied twice, by hand.
There is deliberately no git remote linking the two.

## What differs from the parent

_Nothing yet - this is the unmodified fork point. Record changes here as they are made._

| Change | Date | Notes |
|---|---|---|
| | | |

## Build

PlatformIO / VS Code. Build with `pio run` or `Ctrl+Alt+B`.

```
platform = ststm32@19.7.0
board    = genericSTM32F407VET6
```

**The platform pin is load-bearing.** `ststm32@20.0.0` ships Arduino core 3.0.0,
which migrated to the ArduinoCore-API and breaks this code: `HardwareSerial`
becomes an abstract base class (the concrete class is `Uart`) and
`wiring_constants.h` is removed. Do not unpin without porting the code first.

Output: `.pio/build/genericSTM32F407VET6/firmware.bin`, roughly 34 KB.
Flash at `0x08000000`.

Upload is configured as a custom protocol calling STM32CubeProgrammer CLI over USB
(DFU). The path in `platformio.ini` is machine-specific.

Identify a built binary with:

```
strings firmware.bin | grep FWID
```

The string comes from `include/fw_version.h`. Bump `FW_VARIANT_VER` there when
issuing a build. Note there is no debug serial - `ESPSerial` (PC11/PC10, 115200)
carries the ESP32 protocol, so nothing human-readable can be printed to it.

## Inherited quirks

Carried over from the parent at the fork point. None are blocking, all are traps:

- `platformio.ini` has `lib_extra_dirs = ~/Documents/Arduino/libraries` - a
  machine-local path. Builds will differ between machines depending on what is in
  that folder.
- `src/SGD.cpp` line 1 is `#include "wiring_constants.h"`. Harmless on core 2.12.0,
  but it is the first thing to break if the platform is ever unpinned.
- `HAL_IWDG_MODULE_ENABLED` is defined both in `platformio.ini` build flags and in
  `src/hal_conf_extra.h`, producing a redefinition warning each build.
- `src/TestingCode.cpp` is both `#include`d by the main file and compiled separately
  by PlatformIO. It links today, but it is fragile.
- `src/251212_STM32_WITH_HardwareSERIAL_NOT_420.zip` is a stale archive committed
  into `src/`. Not compiled, but it should not be there.
- Line endings are inconsistent (CRLF/LF), so files can show as fully modified in
  git with no actual change. Check `git diff --stat` before believing a large diff.
- `PA9` is `GasDetection::AnalogueOutput` (`src/SGD.h` line 53). It was once
  borrowed for a debug UART; do not borrow it again without moving the mA output.
