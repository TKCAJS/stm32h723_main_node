#pragma once
#include <stdint.h>
#include <stdbool.h>

// One snapshot of "how is the node", filled once per loop pass in main.cpp and
// handed to every readout: the status LCD and the serial console today, the SD
// logger later. Kept here rather than inside one of them so a second consumer
// does not have to include the first.
//
// Every field is either a number that should be moving or a number that should
// be zero.

typedef struct {
    uint8_t  gear;              // GEAR_UNKNOWN if the rear node has not reported
    bool     gearValid;
    uint16_t rpm;
    bool     armed;             // ARM/SAFE switch — false inhibits shift output
    bool     manualMode;

    // --- clutch, in servo counts over the servo CAN bus ---
    uint16_t clutchCounts;      // meaningless unless clutchFeedbackOk
    bool     clutchFeedbackOk;  // the servo answered recently
    bool     clutchDisengaged;  // past the downshift trigger
    bool     clutchJustEngaged; // inside the bite window
    uint16_t clutchCommanded;   // last target this node sent

    uint16_t hallLeft;
    uint16_t hallRight;

    // --- main bus (rear node) ---
    uint32_t canRxCount;
    uint32_t canGearAgeMs;
    bool     canLive;

    // --- servo bus ---
    uint32_t servoRxCount;
    uint32_t servoTimeouts;     // requests that went unanswered — should stay near 0
    uint32_t servoLatUs;        // last request -> reply
    bool     servoBusOff;

    uint32_t maxLoopUs;         // worst loop pass since boot
    uint32_t inputsDropped;     // queued input events lost — must stay 0
    uint32_t shiftUpCount;      // paddle events seen, so a dead paddle is obvious
    uint32_t shiftDownCount;
    uint32_t neutralCount;
    uint32_t markCount;
} StatusInfo;
