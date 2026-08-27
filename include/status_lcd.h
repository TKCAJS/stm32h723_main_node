#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "node_status.h"

// Onboard WeAct 0.96" ST7735 (160x80) — diagnostics only.
//
// The rider reads the NeoMatrix; this panel exists to answer "is the node
// healthy" with the bike on a stand and no laptop attached. It renders a
// StatusInfo (node_status.h) and owns none of it.


void status_lcd_init();

// Call every loop; internally rate-limited, and only redraws rows whose text
// actually changed so the software SPI cost stays near zero when idle.
void status_lcd_tick(const StatusInfo *s);

// Full-screen message for boot progress and hard faults.
void status_lcd_banner(const char *msg, bool ok);
