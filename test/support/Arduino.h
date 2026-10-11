#pragma once
// Stand-in for the Arduino core in the PC-side tests (pio test -e native). Only
// what gearbox.cpp, paddle.cpp and clutch.cpp actually use. The clock is the
// simulated one in sim.h.

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

uint32_t millis();

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif
