#pragma once
#include <stdint.h>
#include <stdbool.h>

// Calibration persistence — one blob in the last flash sector.
//
// The ESP32 node kept this in NVS. There is no NVS here, and no EEPROM either,
// so the blob goes straight into the last 128 KB sector of the H723's flash.
// The firmware is a few hundred KB at the far end of the same 1 MB, so the two
// cannot meet; if this node ever grows past ~896 KB that stops being true and
// this address has to move.
//
// READ IS FREE. WRITE IS NOT:
//
// Erasing a sector on a single-bank part stalls the CPU for the duration —
// the core cannot fetch instructions from flash while flash is busy, so the
// whole node freezes for the best part of a second. Interrupts are not lost
// (they pend and are serviced late) but nothing else runs. That breaks the one
// rule this node exists to keep, which is why every path to cal_store_write()
// is gated behind ARM/SAFE reading SAFE. Never call it from a control path.

// True if a blob of exactly this length was stored and its header is intact.
// Content validity (version, CRC) is the caller's business.
bool cal_store_read(void *blob, uint32_t len);

// Erase + program. Blocks for the erase; see above. Returns false on any HAL
// error, leaving the sector erased rather than half written — a lost
// calibration is recoverable, a plausible-looking corrupt one is not.
bool cal_store_write(const void *blob, uint32_t len);

// Where the blob lives, for reporting.
uint32_t cal_store_base();
