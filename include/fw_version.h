#ifndef FW_VERSION_H
#define FW_VERSION_H

// Bespoke mA-output variant of the SafeGuard STM32F407VET6 gas detection
// controller. Hard fork - fixes do NOT flow to or from the parent repo.
//
// Parent repo : rpetho/260629-090729-genericSTM32F407VET6
// Forked at   : commit fbc91d6 (2026-09-30)
// Forked on   : 2026-10-07

#define FW_VARIANT_NAME "261007_V6SafeGuardSTM_Bespoke_mAoutput"
#define FW_VARIANT_VER  "0.1.0"
#define FW_FORK_PARENT  "260629-090729-genericSTM32F407VET6@fbc91d6"

// Referenced from a volatile read in setup() so the linker keeps it (--gc-sections
// would otherwise drop it; "used" alone only stops the compiler discarding it).
// Lets a built .bin be
// identified with:  strings firmware.bin | grep FWID
__attribute__((used))
static const char FW_IDENT[] =
    "FWID:" FW_VARIANT_NAME " v" FW_VARIANT_VER " fork-of:" FW_FORK_PARENT;

#endif // FW_VERSION_H
