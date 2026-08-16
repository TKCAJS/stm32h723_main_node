#pragma once
#include <stdint.h>
#include <stdbool.h>

// Onboard WeAct 0.96" ST7735 (160x80) — diagnostics only.
//
// The rider reads the NeoMatrix; this panel exists to answer "is the node
// healthy" with the bike on a stand and no laptop attached. Every line is
// either a number that should be moving or a number that should be zero.

typedef struct {
    uint8_t  gear;            // GEAR_UNKNOWN if the rear node has not reported
    bool     gearValid;
    uint16_t rpm;
    bool     armed;           // ARM/SAFE switch — false inhibits shift output
    bool     manualMode;

    float    clutchVolts;
    bool     clutchDisengaged;
    uint16_t hallLeft;
    uint16_t hallRight;

    uint32_t canRxCount;
    uint32_t canGearAgeMs;
    bool     canLive;

    uint32_t maxLoopUs;       // worst loop pass since boot
    uint32_t inputsDropped;   // queued input events lost — must stay 0
    uint32_t shiftUpCount;    // paddle events seen, so a dead paddle is obvious
    uint32_t shiftDownCount;
    uint32_t neutralCount;
    uint32_t markCount;
} StatusInfo;

void status_lcd_init();

// Call every loop; internally rate-limited, and only redraws rows whose text
// actually changed so the software SPI cost stays near zero when idle.
void status_lcd_tick(const StatusInfo *s);

// Full-screen message for boot progress and hard faults.
void status_lcd_banner(const char *msg, bool ok);
