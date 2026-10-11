#pragma once
#include <stdint.h>
#include <stdbool.h>

// Session log on the onboard microSD (SDMMC1, 4-bit — pins.h).
//
// One CSV text file per boot, T89_0001.CSV upward, readable on any PC:
//
//     ms,type,...
//     1234,BOOT,cal=flash
//     98211,SHIFT,3,4,9150,212,up,1        from,to,rpm,time_ms,kind,ok
//     99870,MARK,2
//
// THE RULE, from pins.h: writes go to a RAM ring and reach the card only when
// the caller says the gearbox is idle. Never inside a shift.
//
// A card write still blocks the loop while it happens — typically a few ms,
// occasionally far longer when the card does its own housekeeping. A paddle
// press that lands in that window is not lost (inputs are interrupt-queued) but
// it is acted on late by however long the write took. `sd` on the console
// shows the worst write seen, which is the number that says whether a given
// card is fit to ride with.
//
// A card that stops answering (pulled, shaken loose) cannot hang the node: the
// library's own waits are unbounded, so they are cut off here after
// SD_LOG_BUSY_LIMIT_MS each, the log is marked dead and the card is never
// touched again until the next boot. The cost is one stall of about a second,
// once.

#define SD_LOG_BUSY_LIMIT_MS   500

typedef struct {
    bool     ready;             // mounted, file open, still healthy
    const char *state;          // "logging", "no card", "open failed", "write failed", "card stopped answering"
    char     file[16];
    uint32_t bytesWritten;
    uint32_t linesDropped;      // RAM ring full — should stay 0
    uint32_t writes;
    uint32_t maxWriteMs;        // worst single card write
    uint32_t maxSyncMs;         // worst directory/FAT sync
    uint16_t pending;           // bytes waiting in RAM
} SdLogStatus;

// Mount the card and open a new session file. May take a moment; boot only.
// False means no logging this session — never a reason to stop the node.
bool sd_log_init();

// Queue one line: "<millis>," is put in front and CRLF behind. printf-style.
// Never blocks, never touches the card. Drops the whole line if RAM is full.
void sd_log_line(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Call every loop pass. At most one card operation per call, and only when
// `idle` is true.
void sd_log_tick(bool idle);

const SdLogStatus *sd_log_status();
