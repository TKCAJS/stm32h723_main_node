#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "serial_cli.h"
#include "servo_can.h"
#include "clutch.h"
#include "cal_store.h"
#include "asmg_md_can.h"

// Output ring. Sized to hold the longest burst the console can produce (a full
// `status` dump is ~700 bytes) so a normal command never loses a line, and to
// drop rather than wait when it cannot keep up.
#define OUT_LEN        4096            // power of two
#define OUT_MASK       (OUT_LEN - 1)
#define DRAIN_BUDGET   128             // bytes per loop pass, so the console can
                                       // never become the thing that stalls loop()
#define RX_PER_PASS    32
#define LINE_MAX       80

static char     _out[OUT_LEN];
static uint16_t _outHead = 0, _outTail = 0;
static uint32_t _outDropped = 0;

static char    _line[LINE_MAX];
static uint8_t _lineLen = 0;

static const StatusInfo *_s = NULL;

static bool     _monOn   = false;
static uint16_t _monMs   = 250;
static uint32_t _monLast = 0;
static bool     _traceOn = false;
static uint32_t _traceSeq = 0;

// ---------------------------------------------------------------------------
// Output — never blocks, drops whole lines under pressure
// ---------------------------------------------------------------------------

static uint16_t _outUsed() { return (uint16_t)((_outHead - _outTail) & OUT_MASK); }

static void _emit(const char *s, uint16_t len) {
    if ((uint16_t)(OUT_LEN - 1 - _outUsed()) < len) { _outDropped++; return; }
    for (uint16_t i = 0; i < len; i++) {
        _out[_outHead] = s[i];
        _outHead = (uint16_t)((_outHead + 1) & OUT_MASK);
    }
}

// Long fixed text goes straight in. _p()'s stack buffer is sized for one
// formatted line, not for a page of help.
static void _puts(const char *s) { _emit(s, (uint16_t)strlen(s)); }

// Attributed so the compiler checks these format strings — a console is
// exactly the kind of code where a wrong conversion never gets exercised.
static void _p(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void _p(const char *fmt, ...) {
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
    _emit(buf, (uint16_t)n);
}

static void _drain() {
    int room = Serial.availableForWrite();
    if (room <= 0) return;
    int budget = DRAIN_BUDGET;

    // Two passes at most: up to the end of the ring, then from the start.
    while (room > 0 && budget > 0 && _outTail != _outHead) {
        uint16_t contiguous = (_outHead > _outTail) ? (uint16_t)(_outHead - _outTail)
                                                    : (uint16_t)(OUT_LEN - _outTail);
        int n = (int)contiguous;
        if (n > room)    n = room;
        if (n > budget)  n = budget;
        Serial.write((const uint8_t *)&_out[_outTail], (size_t)n);
        _outTail = (uint16_t)((_outTail + n) & OUT_MASK);
        room   -= n;
        budget -= n;
    }
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

static const char *_yn(bool b) { return b ? "yes" : "no"; }

static void _print_status() {
    const ServoStatus *v = servo_status();
    const ClutchCal   *c = clutch_cal();

    _p("\r\n-- node ------------------------------------\r\n");
    if (_s) {
        _p("armed        %s  (%s)\r\n", _yn(_s->armed), _s->armed ? "ARMED" : "SAFE");
        _p("mode         %s\r\n", _s->manualMode ? "MANUAL" : "AUTO");
        _p("gear         %u %s   rpm %u\r\n", _s->gear, _s->gearValid ? "" : "(stale)", _s->rpm);
        _p("hall         L %u  R %u\r\n", _s->hallLeft, _s->hallRight);
        _p("loop max     %lu us   inputs dropped %lu\r\n",
           (unsigned long)_s->maxLoopUs, (unsigned long)_s->inputsDropped);
        _p("main bus     rx %lu  gear age %lu ms  live %s\r\n",
           (unsigned long)_s->canRxCount, (unsigned long)_s->canGearAgeMs, _yn(_s->canLive));
    }

    _p("-- servo bus -------------------------------\r\n");
    _p("link         %s   last rx id %08lX\r\n", _yn(v->replyFresh), (unsigned long)v->lastAnyRxId);
    _p("latency      last %lu us  max %lu us\r\n",
       (unsigned long)v->latLastUs, (unsigned long)v->latMaxUs);
    _p("frames       rx %lu  tx %lu  timeouts %lu  txfail %lu\r\n",
       (unsigned long)v->rxCount, (unsigned long)v->txCount,
       (unsigned long)v->timeouts, (unsigned long)v->txFails);
    _p("controller   busoff %s  errpassive %s  tec %lu  rec %lu  recov %lu\r\n",
       _yn(v->busOff), _yn(v->errPassive), (unsigned long)v->busErrTx,
       (unsigned long)v->busErrRx, (unsigned long)v->busRecoveries);
    _p("poll         %s  cmd 0x%02X  %u ms  addr 0x%02X  %s\r\n",
       servo_polling() ? "on" : "off", servo_poll_cmd(), servo_poll_interval_ms(),
       servo_address(), servo_big_endian() ? "big-endian" : "little-endian");
    _p("position     %u  commanded(servo) %u  current %u\r\n",
       v->position, v->cmdPosition, v->current);
    if (v->haveCfg) _p("cfg          torque %u  ilimit %u  reply echoed 0x%02X\r\n",
                       v->torque, v->currentLimit, v->replyEchoCmd04);
    if (v->havePid) _p("pid          P %u  I %u  D %u\r\n", v->pidP, v->pidI, v->pidD);
    if (v->haveAddr) _p("read id      discovered addr 0x%02X\r\n", v->discoveredAddr);

    _p("-- clutch ----------------------------------\r\n");
    _p("feedback     %s\r\n", clutch_feedback_ok() ? "fresh" : "STALE");
    _p("position     %u counts  (%d%% pulled)  commanded %u\r\n",
       clutch_position(), (int)(clutch_pull_fraction() * 100.0f), clutch_commanded());
    _p("state        %s%s\r\n",
       clutch_disengaged() ? "DISENGAGED" : "engaged",
       clutch_just_engaged() ? " (bite window)" : "");
    _p("cal          engaged %u  disengaged %u  trigger %u  bite %u  speed %u\r\n",
       c->engagedCounts, c->disengagedCounts, c->triggerCounts, c->biteCounts, c->moveSpeed);
    if (_outDropped) _p("console      %lu output lines dropped\r\n", (unsigned long)_outDropped);
}

// One line, for `mon`. Everything that moves, nothing that does not.
static void _print_mon() {
    const ServoStatus *v = servo_status();
    _p("clu %5u %3d%% %-10s srv %s lat %5lu to %lu | hall %5u/%5u | loop %lu\r\n",
       clutch_position(), (int)(clutch_pull_fraction() * 100.0f),
       clutch_feedback_ok() ? (clutch_disengaged() ? "DISENGAGED"
                                                   : (clutch_just_engaged() ? "bite" : "engaged"))
                            : "STALE",
       v->replyFresh ? "ok " : "-- ", (unsigned long)v->latLastUs,
       (unsigned long)v->timeouts,
       _s ? _s->hallLeft : 0, _s ? _s->hallRight : 0,
       (unsigned long)(_s ? _s->maxLoopUs : 0));
}

static void _print_trace() {
    ServoTrace t[8];
    uint32_t n = servo_trace_read(&_traceSeq, t, 8);
    for (uint32_t i = 0; i < n; i++) {
        char dir = t[i].dir == SERVO_TRACE_RX ? '<' : (t[i].dir == SERVO_TRACE_TX ? '>' : 'X');
        _p("%8lu %c %08lX %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
           (unsigned long)t[i].ms, dir, (unsigned long)t[i].id,
           t[i].data[0], t[i].data[1], t[i].data[2], t[i].data[3],
           t[i].data[4], t[i].data[5], t[i].data[6], t[i].data[7]);
    }
}

static void _print_help() {
    _puts("\r\n-- readouts --------------------------------\r\n"
       "status              full health dump\r\n"
       "mon [ms|off]        stream one live line (default 250 ms)\r\n"
       "trace on|off        stream raw servo-bus frames\r\n"
       "-- servo bus -------------------------------\r\n"
       "poll on|off         position polling (on at boot)\r\n"
       "pollms <ms>         poll period\r\n"
       "pollcmd 2|7         0x02 pos+cmdpos, 0x07 pos+current\r\n"
       "addr <n>            servo payload address (0x00 factory)\r\n"
       "adopt               take the address from the last readid reply\r\n"
       "endian be|le        16-bit word order (bring-up step 3)\r\n"
       "readid|readpid|readcfg   one-shot reads\r\n"
       "clear               zero counters and latency\r\n");
    _puts("-- clutch (SAFE only) ----------------------\r\n"
       "pos <counts> [spd]  command an absolute servo position\r\n"
       "nudge <+/-delta>    move relative to the last command\r\n"
       "pull <percent>      command by calibrated travel, 0=engaged 100=disengaged\r\n"
       "release             command the engaged limit\r\n"
       "-- calibration (SAFE only) -----------------\r\n"
       "cal                 show it\r\n"
       "cal capture <field> name the CURRENT position: engaged|disengaged|trigger|bite\r\n"
       "cal set <field> <v> engaged|disengaged|trigger|bite|speed\r\n"
       "cal save            write to flash (STALLS the node ~1 s)\r\n"
       "cal load|defaults   re-read flash / factory values\r\n"
       "\r\nDestructive servo commands (save centre, set id, set bitrate,\r\n"
       "factory reset) are deliberately unreachable from here.\r\n");
}

static void _print_cal() {
    const ClutchCal *c = clutch_cal();
    _p("engaged     %5u   (clutch fully engaged, paddle idle)\r\n", c->engagedCounts);
    _p("disengaged  %5u   (clutch fully pulled)\r\n", c->disengagedCounts);
    _p("trigger     %5u   (downshift gate)\r\n", c->triggerCounts);
    _p("bite        %5u   (engaged-side edge of the bite window)\r\n", c->biteCounts);
    _p("speed       %5u   (0 fastest .. 1280 slowest)\r\n", c->moveSpeed);
    _p("live        %5u   feedback %s\r\n", clutch_position(),
       clutch_feedback_ok() ? "fresh" : "STALE");
    _p("stored at   0x%08lX\r\n", (unsigned long)cal_store_base());
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------

// Every write path comes through here. The rule is the README's: the tablet
// (and now the terminal) may look at any time, but may only change things with
// ARM/SAFE reading SAFE.
static bool _mayWrite() {
    if (_s && _s->armed) {
        _p("refused: ARM/SAFE reads ARMED. Flip it to SAFE first.\r\n");
        return false;
    }
    return true;
}

static bool _num(const char *tok, long *out) {
    if (!tok || !*tok) return false;
    char *end = NULL;
    long v = strtol(tok, &end, 0);      // 0x.. accepted, for addresses
    if (end == tok || *end != '\0') return false;
    *out = v;
    return true;
}

static void _dispatch(char *line) {
    char *argv[4] = { NULL, NULL, NULL, NULL };
    int argc = 0;
    for (char *p = line; *p && argc < 4; ) {
        while (*p == ' ') p++;
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ') p++;
        if (*p) *p++ = '\0';
    }
    if (argc == 0) return;

    const char *cmd = argv[0];
    long n = 0;

    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) { _print_help(); return; }
    if (!strcmp(cmd, "status"))                    { _print_status(); return; }

    if (!strcmp(cmd, "mon")) {
        if (argc > 1 && !strcmp(argv[1], "off")) { _monOn = false; _p("mon off\r\n"); return; }
        if (argc > 1 && _num(argv[1], &n) && n >= 20) _monMs = (uint16_t)n;
        _monOn = true;
        _p("mon every %u ms — any command stops it\r\n", _monMs);
        return;
    }

    if (!strcmp(cmd, "trace")) {
        _traceOn = (argc > 1 && !strcmp(argv[1], "on"));
        _traceSeq = servo_trace_seq();           // start from now, not from history
        _p("trace %s\r\n", _traceOn ? "on" : "off");
        return;
    }

    if (!strcmp(cmd, "poll")) {
        servo_set_polling(argc > 1 && !strcmp(argv[1], "on"));
        _p("poll %s\r\n", servo_polling() ? "on" : "off");
        return;
    }

    if (!strcmp(cmd, "pollms")) {
        if (!_num(argv[1], &n)) { _p("usage: pollms <ms>\r\n"); return; }
        servo_set_poll_interval_ms((uint16_t)n);
        _p("poll period %u ms\r\n", servo_poll_interval_ms());
        return;
    }

    if (!strcmp(cmd, "pollcmd")) {
        if (!_num(argv[1], &n) || (n != 2 && n != 7)) { _p("usage: pollcmd 2|7\r\n"); return; }
        servo_set_poll_cmd((uint8_t)n);
        _p("poll cmd 0x%02X\r\n", servo_poll_cmd());
        return;
    }

    if (!strcmp(cmd, "addr")) {
        if (!_num(argv[1], &n) || n < 0 || n > 0xFF) { _p("usage: addr <0..255>\r\n"); return; }
        servo_set_address((uint8_t)n);
        _p("addr 0x%02X\r\n", servo_address());
        return;
    }

    if (!strcmp(cmd, "adopt")) {
        const ServoStatus *v = servo_status();
        if (!v->haveAddr) { _p("no readid reply yet — run readid first\r\n"); return; }
        servo_set_address(v->discoveredAddr);
        _p("addr 0x%02X (adopted)\r\n", servo_address());
        return;
    }

    if (!strcmp(cmd, "endian")) {
        if (argc < 2) { _p("usage: endian be|le\r\n"); return; }
        servo_set_big_endian(!strcmp(argv[1], "be"));
        _p("word order %s\r\n", servo_big_endian() ? "big-endian" : "little-endian");
        return;
    }

    if (!strcmp(cmd, "readid"))  { servo_request_read_id();  _p("0xFD queued\r\n"); return; }
    if (!strcmp(cmd, "readpid")) { servo_request_read_pid(); _p("0x06 queued\r\n"); return; }
    if (!strcmp(cmd, "readcfg")) { servo_request_read_cfg(); _p("0x04 queued\r\n"); return; }
    if (!strcmp(cmd, "clear"))   { servo_clear_counters(); _outDropped = 0; _p("counters cleared\r\n"); return; }

    if (!strcmp(cmd, "pos")) {
        if (!_mayWrite()) return;
        if (!_num(argv[1], &n) || n < 0 || n > asmg::kPositionMax) {
            _p("usage: pos <0..%u> [speed]\r\n", (unsigned)asmg::kPositionMax);
            return;
        }
        // Deliberately NOT clutch_command_counts(): this is the bench slider,
        // and finding a worn clutch's real stop means going outside the
        // currently calibrated travel to look for it.
        long spd = clutch_cal()->moveSpeed;
        if (argc > 2) _num(argv[2], &spd);
        servo_command_position((uint16_t)n, (uint16_t)spd);
        _p("pos %ld speed %ld (raw, unclamped by calibration)\r\n", n, spd);
        return;
    }

    if (!strcmp(cmd, "nudge")) {
        if (!_mayWrite()) return;
        if (!_num(argv[1], &n)) { _p("usage: nudge <+/-delta>\r\n"); return; }
        long t = (long)clutch_commanded() + n;
        if (t < 0) t = 0;
        if (t > asmg::kPositionMax) t = asmg::kPositionMax;
        clutch_command_counts((uint16_t)t);
        _p("nudge -> %u (clamped to calibrated travel)\r\n", clutch_commanded());
        return;
    }

    if (!strcmp(cmd, "pull")) {
        if (!_mayWrite()) return;
        if (!_num(argv[1], &n) || n < 0 || n > 100) { _p("usage: pull <0..100>\r\n"); return; }
        clutch_command_pull((float)n / 100.0f);
        _p("pull %ld%% -> %u counts\r\n", n, clutch_commanded());
        return;
    }

    if (!strcmp(cmd, "release")) {
        if (!_mayWrite()) return;
        clutch_release();
        _p("release -> %u counts\r\n", clutch_commanded());
        return;
    }

    if (!strcmp(cmd, "cal")) {
        if (argc == 1) { _print_cal(); return; }

        if (!_mayWrite()) return;

        const char *err = "";
        if (!strcmp(argv[1], "load")) {
            // Swapping the live calibration is a change like any other, even
            // though the numbers come off this node's own flash.
            _p(clutch_cal_load() ? "loaded from flash\r\n"
                                 : "nothing valid in flash — calibration unchanged\r\n");
            return;
        }
        if (!strcmp(argv[1], "defaults")) { clutch_cal_defaults(); _p("factory values loaded (not saved)\r\n"); return; }
        if (!strcmp(argv[1], "save")) {
            // Stalls the node for the erase — see cal_store.h. Only reachable
            // here, and only while SAFE.
            _p(clutch_cal_save() ? "saved to flash\r\n" : "SAVE FAILED — flash unchanged or erased\r\n");
            return;
        }
        if (!strcmp(argv[1], "capture")) {
            if (argc < 3) { _p("usage: cal capture engaged|disengaged|trigger|bite\r\n"); return; }
            if (clutch_cal_capture(argv[2], &err)) _p("%s = %u (captured)\r\n", argv[2], clutch_position());
            else                                   _p("rejected: %s\r\n", err);
            return;
        }
        if (!strcmp(argv[1], "set")) {
            if (argc < 4 || !_num(argv[3], &n)) { _p("usage: cal set <field> <value>\r\n"); return; }
            if (clutch_cal_set(argv[2], n, &err)) _p("%s = %ld\r\n", argv[2], n);
            else                                  _p("rejected: %s\r\n", err);
            return;
        }
        _p("usage: cal [capture|set|save|load|defaults]\r\n");
        return;
    }

    _p("unknown: %s  (try help)\r\n", cmd);
}

// ---------------------------------------------------------------------------
// Public
// ---------------------------------------------------------------------------

void serial_cli_init() {
    _p("\r\nT89 main node — servo/clutch console. 'help' for commands.\r\n");
}

void serial_cli_tick(const StatusInfo *s) {
    _s = s;

    // Input, bounded. A host pasting a script cannot hold the loop open.
    for (int i = 0; i < RX_PER_PASS && Serial.available() > 0; i++) {
        int c = Serial.read();
        if (c < 0 || c == '\r') continue;
        if (c == '\n') {
            _line[_lineLen] = '\0';
            _emit("\r\n", 2);
            if (_lineLen) {
                _monOn = false;         // any command stops the stream
                _dispatch(_line);
            }
            _lineLen = 0;
            continue;
        }
        if (c == 8 || c == 127) { if (_lineLen) { _lineLen--; _emit("\b \b", 3); } continue; }
        if (c >= ' ' && _lineLen < LINE_MAX - 1) {
            _line[_lineLen++] = (char)c;
            char e = (char)c;
            _emit(&e, 1);               // local echo: most terminals do not
        }
    }

    if (_traceOn) _print_trace();

    if (_monOn && (millis() - _monLast) >= _monMs) {
        _monLast = millis();
        _print_mon();
    }

    _drain();
}
