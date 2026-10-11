#include <Arduino.h>
#include <STM32SD.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "sd_log.h"
#include "pins.h"

#define RING_LEN        4096        // power of two
#define RING_MASK       (RING_LEN - 1)
#define CHUNK           512         // one sector per write
#define FLUSH_AFTER_MS  2000        // a part-filled chunk is written after this long
#define SYNC_EVERY_MS   5000        // directory entry / FAT brought up to date this often
#define MAX_SESSIONS    9999

static char     _ring[RING_LEN];
static uint16_t _head = 0, _tail = 0;

static File        _file;
static SdLogStatus _st = { false, "not started", "", 0, 0, 0, 0, 0, 0 };
static bool        _dirty     = false;
static uint32_t    _lastWrite = 0;
static uint32_t    _lastSync  = 0;

// ---------------------------------------------------------------------------
// Bounding the library's waits
// ---------------------------------------------------------------------------
// Left alone, the SD stack can wait on the card for ever, in two places:
//
//   1. FatFs's sd_diskio.c passes SDMMC_DATATIMEOUT as the transfer timeout,
//      and on this chip that is 0xFFFFFFFF ms.
//   2. It then ends every sector read and write with
//          while (BSP_SD_GetCardState() != MSD_OK) {}
//      and nothing else.
//
// A card that stops answering mid-ride would park the whole node in either
// one. The linker wraps all three calls (platformio.ini, --wrap) so they come
// through here: a transfer gets SD_LOG_BUSY_LIMIT_MS, and a card busy for
// longer than that is reported as ready, which ends the loop. Either way the
// log is marked dead and the card is not touched again.
static volatile bool _cardStuck  = false;
static uint32_t      _busySince  = 0;
static bool          _busyTiming = false;

extern "C" uint8_t __real_BSP_SD_GetCardState(void);

extern "C" uint8_t __wrap_BSP_SD_GetCardState(void) {
    if (_cardStuck) return 0;
    uint8_t s = __real_BSP_SD_GetCardState();
    if (s == 0) { _busyTiming = false; return 0; }

    if (!_busyTiming) { _busyTiming = true; _busySince = millis(); }
    if ((millis() - _busySince) >= SD_LOG_BUSY_LIMIT_MS) {
        _cardStuck = true;
        return 0;
    }
    return s;
}

extern "C" uint8_t __real_BSP_SD_WriteBlocks(uint32_t *data, uint32_t addr, uint32_t blocks, uint32_t timeout);
extern "C" uint8_t __real_BSP_SD_ReadBlocks(uint32_t *data, uint32_t addr, uint32_t blocks, uint32_t timeout);

extern "C" uint8_t __wrap_BSP_SD_WriteBlocks(uint32_t *data, uint32_t addr, uint32_t blocks, uint32_t) {
    if (_cardStuck) return 1;
    uint8_t r = __real_BSP_SD_WriteBlocks(data, addr, blocks, SD_LOG_BUSY_LIMIT_MS);
    if (r != 0) _cardStuck = true;
    return r;
}

extern "C" uint8_t __wrap_BSP_SD_ReadBlocks(uint32_t *data, uint32_t addr, uint32_t blocks, uint32_t) {
    if (_cardStuck) return 1;
    uint8_t r = __real_BSP_SD_ReadBlocks(data, addr, blocks, SD_LOG_BUSY_LIMIT_MS);
    if (r != 0) _cardStuck = true;
    return r;
}

static void _fail(const char *why) {
    // Deliberately no close(): that would be one more card operation on a
    // card that has just shown it cannot be trusted with one.
    _st.ready = false;
    _st.state = why;
}

// ---------------------------------------------------------------------------
// RAM ring
// ---------------------------------------------------------------------------

static uint16_t _used() { return (uint16_t)((_head - _tail) & RING_MASK); }

void sd_log_line(const char *fmt, ...) {
    if (!_st.ready) return;

    char buf[128];
    int n = snprintf(buf, sizeof(buf), "%lu,", (unsigned long)millis());
    va_list ap;
    va_start(ap, fmt);
    int m = vsnprintf(buf + n, sizeof(buf) - n - 2, fmt, ap);
    va_end(ap);
    if (m < 0) return;
    n += m;
    if (n > (int)sizeof(buf) - 2) n = sizeof(buf) - 2;
    buf[n++] = '\r';
    buf[n++] = '\n';

    if ((uint16_t)(RING_LEN - 1 - _used()) < n) { _st.linesDropped++; return; }
    for (int i = 0; i < n; i++) {
        _ring[_head] = buf[i];
        _head = (uint16_t)((_head + 1) & RING_MASK);
    }
}

// ---------------------------------------------------------------------------
// Card
// ---------------------------------------------------------------------------

bool sd_log_init() {
    // All six named: left to its defaults the library takes the FIRST pin the
    // core lists for each signal, and for D0 that is PB13 — FDCAN2_TX.
    SD.setDx(PIN_SD_D0, PIN_SD_D1, PIN_SD_D2, PIN_SD_D3);
    SD.setCK(PIN_SD_CK);
    SD.setCMD(PIN_SD_CMD);

    if (!SD.begin(SD_DETECT_NONE)) { _st.state = "no card"; return false; }

    // No clock on this node, so sessions are numbered: the first free name.
    for (uint16_t n = 1; n <= MAX_SESSIONS; n++) {
        snprintf(_st.file, sizeof(_st.file), "T89_%04u.CSV", n);
        if (!SD.exists(_st.file)) break;
    }

    _file = SD.open(_st.file, FILE_WRITE);
    if (!_file || _cardStuck) { _st.state = "open failed"; return false; }

    _st.ready = true;
    _st.state = "logging";
    _lastWrite = _lastSync = millis();

    const char *hdr = "ms,type,a,b,c,d,e,f\r\n";
    for (const char *p = hdr; *p; p++) { _ring[_head] = *p; _head = (uint16_t)((_head + 1) & RING_MASK); }
    return true;
}

void sd_log_tick(bool idle) {
    _st.pending = _used();
    if (!_st.ready || !idle) return;

    uint32_t now = millis();

    if (_used() >= CHUNK || (_used() && (now - _lastWrite) >= FLUSH_AFTER_MS)) {
        static uint8_t chunk[CHUNK];
        uint16_t n = _used() < CHUNK ? _used() : CHUNK;
        for (uint16_t i = 0; i < n; i++) chunk[i] = (uint8_t)_ring[(_tail + i) & RING_MASK];

        size_t wrote = _file.write(chunk, n);
        uint32_t took = millis() - now;
        if (took > _st.maxWriteMs) _st.maxWriteMs = took;

        if (_cardStuck)      { _fail("card stopped answering"); return; }
        if (wrote != n)      { _fail("write failed");           return; }

        _tail = (uint16_t)((_tail + n) & RING_MASK);
        _st.bytesWritten += n;
        _st.writes++;
        _dirty = true;
        _lastWrite = millis();
        return;                     // one card operation per pass
    }

    if (_dirty && (now - _lastSync) >= SYNC_EVERY_MS) {
        _file.flush();
        uint32_t took = millis() - now;
        if (took > _st.maxSyncMs) _st.maxSyncMs = took;
        if (_cardStuck) { _fail("card stopped answering"); return; }
        _dirty = false;
        _lastSync = millis();
    }
}

const SdLogStatus *sd_log_status() {
    _st.pending = _used();
    return &_st;
}
