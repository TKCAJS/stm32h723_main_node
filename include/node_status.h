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
    bool     engineRunning;     // rpm at or above ENGINE_RUNNING_RPM — `cal save` is deferred
    bool     manualMode;

    // --- gearbox state machine ---
    uint8_t  gbState;           // GearboxState
    bool     gbShifting;        // shifting, or waiting for the clutch
    char     gbLetter;          // last shift letter U D N C, 0 if none yet
    uint32_t gbAborts;          // downshifts refused: clutch never confirmed — should stay 0

    // --- clutch, in servo counts over the servo CAN bus ---
    uint16_t clutchCounts;      // meaningless unless clutchFeedbackOk
    bool     clutchFeedbackOk;  // the servo answered recently
    bool     clutchDisengaged;  // past the downshift trigger
    bool     clutchJustEngaged; // inside the bite window
    uint16_t clutchCommanded;   // last target this node sent

    uint16_t hallLeft;
    uint16_t hallRight;
    uint16_t paddlePos;         // 0..10000, the further-pulled paddle

    // --- main bus (rear node) ---
    uint32_t canRxCount;
    uint32_t canGearAgeMs;
    bool     canLive;

    // --- servo bus ---
    uint32_t servoRxCount;
    uint32_t servoTimeouts;     // requests that went unanswered — should stay near 0
    uint32_t servoLatUs;        // last request -> reply
    bool     servoBusOff;

    // --- SD session log ---
    bool        sdReady;        // mounted, file open, still healthy
    const char *sdState;        // "logging", "no card", ... (sd_log.h)
    const char *sdFile;         // "T89_0001.CSV"
    uint32_t    sdBytes;        // written to the card so far
    uint32_t    sdWorstMs;      // longest single card write or sync

    uint32_t maxLoopUs;         // worst loop pass since boot
    uint32_t inputsDropped;     // queued input events lost — must stay 0
    uint32_t shiftUpCount;      // paddle events seen, so a dead paddle is obvious
    uint32_t shiftDownCount;
    uint32_t neutralCount;
    uint32_t markCount;
} StatusInfo;
