// Onboard 0.96" ST7735 (160x80) on the WeAct H723 board — diagnostics readout.
// CS=PE11, SCK=PE12, DC=PE13, MOSI=PE14, BLK=PE10; panel RST is tied to NRST.
// Software SPI on a dedicated pin set, as on the transmitter node: the text
// refresh is a few Hz, so the bit-bang cost is negligible and it can never
// contend with another bus.

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <stdio.h>
#include <string.h>
#include "status_lcd.h"
#include "pins.h"
#include "can_ids.h"

#define REFRESH_MS 250      // 4 Hz — fast enough to watch a paddle register

#define C_BLACK  0x0000
#define C_WHITE  0xFFFF
#define C_GREEN  0x07E0
#define C_RED    0xF800
#define C_YELLOW 0xFFE0
#define C_GRAY   0x8410

// 0.96" 80x160 IPS panel: column offset 26, row offset 1, BGR order.
// Rotation 1 -> 160 wide x 80 tall.
static Arduino_SWSPI  _bus(PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_SCK, PIN_LCD_MOSI, GFX_NOT_DEFINED);
static Arduino_ST7735 _lcd(&_bus, GFX_NOT_DEFINED /* RST on NRST */, 1,
                           true /* IPS */, 80, 160, 26, 1, 26, 1,
                           true /* BGR */);

// 7 rows of size-1 text (6x8 glyphs), 26 characters wide. The servo bus needs
// a line of its own, so the rows are pitched at 11 px instead of 13 — an 8 px
// glyph with 3 px of air, and the last baseline at y=68 still inside 80.
#define ROWS       7
#define ROW_CHARS  27          // 26 + NUL
#define ROW_Y(i)   (2 + (i) * 11)

static uint32_t _lastRefresh = 0;
static char     _shown[ROWS][ROW_CHARS];   // last text actually drawn, per row

static void _row(int i, uint16_t color, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

static void _row(int i, uint16_t color, const char *fmt, ...) {
    char buf[ROW_CHARS];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    // Pad to full width so shorter text overwrites stale characters.
    if (n < 0) n = 0;
    for (int p = n; p < (int)sizeof(buf) - 1; p++) buf[p] = ' ';
    buf[sizeof(buf) - 1] = '\0';

    // Software SPI is slow enough that redrawing unchanged rows is worth
    // avoiding; most rows are static most of the time.
    if (strcmp(buf, _shown[i]) == 0) return;
    strcpy(_shown[i], buf);

    _lcd.setCursor(3, ROW_Y(i));
    _lcd.setTextColor(color, C_BLACK);
    _lcd.print(buf);
}

static char _gear_char(uint8_t gear, bool valid) {
    if (!valid || gear == GEAR_UNKNOWN) return '?';
    if (gear == GEAR_NEUTRAL) return 'N';
    return (char)('0' + gear);
}

void status_lcd_init() {
    pinMode(PIN_LCD_BLK, OUTPUT);
    digitalWrite(PIN_LCD_BLK, LOW);   // backlight on (flip to HIGH if it stays dark)

    _lcd.begin();
    _lcd.fillScreen(C_BLACK);
    _lcd.setTextSize(1);
    memset(_shown, 0, sizeof(_shown));
}

void status_lcd_banner(const char *msg, bool ok) {
    _lcd.fillScreen(C_BLACK);
    _lcd.setTextColor(ok ? C_GREEN : C_RED, C_BLACK);
    _lcd.setCursor(3, 36);
    _lcd.print(msg);
    memset(_shown, 0, sizeof(_shown));   // force a full redraw on the next tick
}

void status_lcd_tick(const StatusInfo *s) {
    uint32_t now = millis();
    if (now - _lastRefresh < REFRESH_MS) return;
    _lastRefresh = now;

    // Line 1: what the box is doing. Red gear when the rear node is not
    // reporting — that is the one field a wrong answer would be dangerous.
    _row(0, s->gearValid ? C_WHITE : C_RED,
         "G%c %5u rpm %s", _gear_char(s->gear, s->gearValid), s->rpm,
         s->armed ? (s->manualMode ? "MAN" : "AUT") : "SAFE");

    // Line 2: clutch position, in the servo's own counts, straight from the
    // servo's CAN reply. STALE is a state the analog path could not report —
    // a dead ADS1115 read looked exactly like a clutch holding still.
    if (!s->clutchFeedbackOk) {
        _row(1, C_RED, "CLU ----- STALE");
    } else {
        _row(1, C_WHITE, "CLU %5u %s", s->clutchCounts,
             s->clutchDisengaged ? "DISENG" : (s->clutchJustEngaged ? "BITE" : "ENGAGED"));
    }

    // Line 3: servo bus health. Timeouts climbing means the servo stopped
    // answering; BUSOFF means the controller took itself off the bus.
    if (s->servoBusOff) {
        _row(2, C_RED, "SRV BUSOFF to%lu", (unsigned long)s->servoTimeouts);
    } else {
        _row(2, s->clutchFeedbackOk ? C_GREEN : C_YELLOW, "SRV rx%lu to%lu %luus",
             (unsigned long)s->servoRxCount, (unsigned long)s->servoTimeouts,
             (unsigned long)(s->servoLatUs > 99999 ? 99999 : s->servoLatUs));
    }

    // Line 4: raw paddle halls, for setting the travel range.
    _row(3, C_GRAY, "HALL L%5u R%5u", s->hallLeft, s->hallRight);

    // Line 5: main bus health. Gear age climbing means the rear node went quiet.
    _row(4, (s->canLive && s->canGearAgeMs < 1000) ? C_GREEN : C_RED,
         "CAN rx%lu age%lums", (unsigned long)s->canRxCount,
         (unsigned long)(s->canGearAgeMs > 9999 ? 9999 : s->canGearAgeMs));

    // Line 6: the numbers that carry the whole point of this node. maxLoopUs
    // is what the ESP32 could not keep small, and drop must never leave 0.
    _row(5, (s->inputsDropped == 0 && s->maxLoopUs < 20000) ? C_GREEN : C_RED,
         "LOOP %luus drop%lu", (unsigned long)s->maxLoopUs,
         (unsigned long)s->inputsDropped);

    // Line 7: paddle event counters — press a paddle, watch it move. A paddle
    // that never increments is a wiring fault, not a firmware mystery.
    _row(6, C_YELLOW, "U%lu D%lu N%lu M%lu",
         (unsigned long)s->shiftUpCount, (unsigned long)s->shiftDownCount,
         (unsigned long)s->neutralCount, (unsigned long)s->markCount);
}
