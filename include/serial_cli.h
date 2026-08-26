#pragma once
#include "node_status.h"

// Serial console — what the web interface used to be.
//
// The ESP32 node served WebInterface.h over its own WiFi AP: live readouts,
// a servo slider, and buttons that captured the current clutch voltage as a
// threshold. None of that can run here. This node has no radio by design (the
// whole reason for the migration — see README), so there is no server, and a
// pit-lane Android tablet cannot reach a USB serial port either.
//
// So the calibration surface splits in two, as the README lays out:
//   - the tablet path stays with the dash, which proxies get/set over CAN;
//   - this console is the always-available one. A terminal on a laptop, or a
//     USB-serial adapter on the bench, and every knob the web page had.
//
// It also carries the ASMG sniffer's controls — poll rate, address, word
// order, raw frame trace — because they are the same knobs, and the bench work
// that proved the servo protocol has to stay repeatable on the real node.
//
// Non-blocking, both directions. Input is drained a bounded number of bytes per
// pass; output goes into a ring that is emptied only as fast as the UART will
// take it without waiting. If the ring fills, whole lines are dropped and
// counted — a console is never allowed to stall the shift path, and a
// half-written line is worse than a missing one.
//
// Every command that MOVES something or CHANGES something is refused unless
// ARM/SAFE reads SAFE. Nobody re-times a clutch from a terminal mid-session.

void serial_cli_init();

// Call every loop. Reads what has arrived, writes what fits.
void serial_cli_tick(const StatusInfo *s);
