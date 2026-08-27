#include <Arduino.h>
#include <string.h>
#include "servo_can.h"
#include "can_main.h"
#include "asmg_md_can.h"

// Reply deadline. The integration brief targets 20 ms; the bench sniffer used
// 50 ms because a diagnosis tool would rather wait than lie. In the control
// path the opposite is true — a late reply is a stale position, and the poll
// after it is only 20 ms away.
#define REPLY_TIMEOUT_US    20000

// Floor between consecutive 0x01 position frames. The mailbox always holds the
// newest target, so a caller commanding every loop pass is coalesced to this
// rate rather than flooding the bus. 5 ms is ~1.3 % bus load at 250 kbit/s.
#define POS_MIN_INTERVAL_MS 5

#define POLL_DEFAULT_MS     20      // 50 Hz — as the bench tool polled
#define TRACE_LEN           64      // power of two

static FDCAN_HandleTypeDef _hfdcan;
static bool _ok = false;

// --- configuration (bench-settable from the serial console) ---
static uint8_t  _addr       = asmg::kAddrFactoryDefault;
static bool     _bigEndian  = ASMG_WORD_BIG_ENDIAN;
static bool     _polling    = true;    // ON at boot: unlike the bench tool this
                                       // is the clutch's only position sense
static uint8_t  _pollCmd    = (uint8_t)asmg::Cmd::ReadPositionAndI;
static uint16_t _pollMs     = POLL_DEFAULT_MS;

// --- one-shot request mailbox ---
static bool     _reqReadId  = false;
static bool     _reqReadPid = false;
static bool     _reqReadCfg = false;
static bool     _posPending = false;
static uint16_t _posCounts  = 0;
static uint16_t _posSpeed   = 0;
static uint32_t _lastPosMs  = 0;

// --- outstanding request, strictly one at a time ---
static bool     _waiting    = false;
static uint8_t  _waitCmd    = 0;
static uint8_t  _waitAddr   = 0;
static uint32_t _sentUs     = 0;
static uint32_t _lastPollMs = 0;

static ServoStatus _st = {};

static ServoTrace _trace[TRACE_LEN];
static uint32_t   _traceSeq = 0;      // seq of the NEXT entry to be written

// ---------------------------------------------------------------------------
// Word order
// ---------------------------------------------------------------------------
// The codec packs at compile time; this bus can be told to flip at runtime
// because the manufacturer never documented the byte order and the answer is a
// bench observation (bring-up step 3). Once it is settled, bake it into
// ASMG_WORD_BIG_ENDIAN and the two agree again.
static void _pack16(uint8_t *p, uint16_t v) {
    if (_bigEndian) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)(v & 0xFF); }
    else            { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
}

static uint16_t _unpack16(const uint8_t *p) {
    return _bigEndian ? (uint16_t)(((uint16_t)p[0] << 8) | p[1])
                      : (uint16_t)(((uint16_t)p[1] << 8) | p[0]);
}

static void _trace_push(uint8_t dir, uint32_t id, uint8_t dlc, const uint8_t *data) {
    ServoTrace *e = &_trace[_traceSeq & (TRACE_LEN - 1)];
    e->seq = _traceSeq++;
    e->ms  = millis();
    e->id  = id;
    e->dlc = dlc;
    e->dir = dir;
    memcpy(e->data, data, 8);
}

// ---------------------------------------------------------------------------
// Transport
// ---------------------------------------------------------------------------

static bool _tx(const asmg::Frame &f) {
    bool ok = false;

    if (_ok && HAL_FDCAN_GetTxFifoFreeLevel(&_hfdcan) > 0) {
        FDCAN_TxHeaderTypeDef tx = {};
        tx.Identifier          = f.id;
        tx.IdType              = FDCAN_EXTENDED_ID;
        tx.TxFrameType         = FDCAN_DATA_FRAME;
        tx.DataLength          = FDCAN_DLC_BYTES_8;
        tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx.BitRateSwitch       = FDCAN_BRS_OFF;
        tx.FDFormat            = FDCAN_CLASSIC_CAN;
        tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        ok = HAL_FDCAN_AddMessageToTxFifoQ(&_hfdcan, &tx, (uint8_t *)f.data) == HAL_OK;
    }

    _trace_push(ok ? SERVO_TRACE_TX : SERVO_TRACE_TX_FAIL, f.id, f.dlc, f.data);
    if (ok) _st.txCount++; else _st.txFails++;
    return ok;
}

bool servo_can_init() {
    // Both FDCAN instances share one kernel clock, so this is the same setting
    // can_init() applies and re-applying it is harmless — which keeps the two
    // init functions independent of each other's call order.
    RCC_PeriphCLKInitTypeDef pclk = {};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    pclk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) return false;

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {};
    g.Pin       = GPIO_PIN_12 | GPIO_PIN_13;  // PB12 = FDCAN2_RX, PB13 = FDCAN2_TX
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &g);

    _hfdcan.Instance        = FDCAN2;
    _hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    _hfdcan.Init.Mode        = FDCAN_MODE_NORMAL;
    // Single-shot, deliberately. With retransmission on, a servo that is off or
    // unplugged never ACKs, so every frame is retried until the error counters
    // force bus-off — a retry storm that buys nothing on a two-node bus. Off, a
    // missing servo simply shows up as timeouts climbing, which is the truth.
    _hfdcan.Init.AutoRetransmission = DISABLE;
    _hfdcan.Init.TransmitPause      = DISABLE;
    _hfdcan.Init.ProtocolException  = DISABLE;
    // 136 MHz PLL1Q / prescaler 16 = 8.5 MHz; 34 tq/bit (1 + 26 + 7) = 250 kbit/s,
    // sample point 79.4 % — the same shape as the main bus, one prescaler step
    // slower for the servo's factory rate.
    _hfdcan.Init.NominalPrescaler     = 16;
    _hfdcan.Init.NominalSyncJumpWidth = 7;
    _hfdcan.Init.NominalTimeSeg1      = 26;
    _hfdcan.Init.NominalTimeSeg2      = 7;
    _hfdcan.Init.DataPrescaler        = 16;
    _hfdcan.Init.DataSyncJumpWidth    = 7;
    _hfdcan.Init.DataTimeSeg1         = 26;
    _hfdcan.Init.DataTimeSeg2         = 7;
    // FDCAN1 and FDCAN2 share ONE message RAM. FDCAN1 is at offset 0 and
    // occupies CAN_MAIN_MSG_RAM_WORDS 32-bit words; overlap them and the two
    // buses quietly corrupt each other's mailboxes. Change FDCAN1's element
    // counts and this offset has to move with them.
    _hfdcan.Init.MessageRAMOffset    = CAN_MAIN_MSG_RAM_WORDS;
    // No hardware filters: everything non-matching is accepted below, so the
    // reply arbitration ID can be recorded rather than assumed.
    _hfdcan.Init.StdFiltersNbr       = 0;
    _hfdcan.Init.ExtFiltersNbr       = 0;
    _hfdcan.Init.RxFifo0ElmtsNbr     = 16;
    _hfdcan.Init.RxFifo0ElmtSize     = FDCAN_DATA_BYTES_8;
    _hfdcan.Init.RxFifo1ElmtsNbr     = 0;
    _hfdcan.Init.RxFifo1ElmtSize     = FDCAN_DATA_BYTES_8;
    _hfdcan.Init.RxBuffersNbr        = 0;
    _hfdcan.Init.RxBufferSize        = FDCAN_DATA_BYTES_8;
    _hfdcan.Init.TxEventsNbr         = 0;
    _hfdcan.Init.TxBuffersNbr        = 0;
    _hfdcan.Init.TxFifoQueueElmtsNbr = 8;
    _hfdcan.Init.TxFifoQueueMode     = FDCAN_TX_FIFO_OPERATION;
    _hfdcan.Init.TxElmtSize          = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(&_hfdcan) != HAL_OK) return false;

    if (HAL_FDCAN_ConfigGlobalFilter(&_hfdcan,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,   // non-matching standard
                                     FDCAN_ACCEPT_IN_RX_FIFO0,   // non-matching extended
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
        return false;

    if (HAL_FDCAN_Start(&_hfdcan) != HAL_OK) return false;

    _ok = true;
    return true;
}

// ---------------------------------------------------------------------------
// Health
// ---------------------------------------------------------------------------

static void _check_health() {
    FDCAN_ProtocolStatusTypeDef ps;
    FDCAN_ErrorCountersTypeDef  ec;

    HAL_FDCAN_GetProtocolStatus(&_hfdcan, &ps);
    HAL_FDCAN_GetErrorCounters(&_hfdcan, &ec);

    _st.busOff     = (ps.BusOff != 0);
    _st.errPassive = (ps.ErrorPassive != 0);
    _st.busErrTx   = ec.TxErrorCnt;
    _st.busErrRx   = ec.RxErrorCnt;

    if (!_st.busOff) return;

    // Bus-off sets CCCR.INIT and the controller stays there until INIT is
    // cleared; the hardware then counts 129 idle sequences and rejoins on its
    // own. Clearing the bit directly rather than re-running HAL_FDCAN_Start()
    // avoids having to lie to the HAL about its own state — Start() refuses
    // once the handle is BUSY, which it still is.
    CLEAR_BIT(_hfdcan.Instance->CCCR, FDCAN_CCCR_INIT);
    _st.busRecoveries++;
}

// ---------------------------------------------------------------------------
// Reply handling
// ---------------------------------------------------------------------------

static void _handle_rx(uint32_t id, const uint8_t *d) {
    _st.lastAnyRxId = id;
    _st.rxCount++;

    if (!_waiting) return;   // unsolicited — visible in the trace, not decoded

    // 0xFD went to the broadcast address, so byte 0 is the discovery answer
    // rather than an echo of what was polled. Everything else must echo.
    bool addrOk = (_waitCmd == (uint8_t)asmg::Cmd::ReadId) || (d[0] == _waitAddr);
    // The manufacturer doc shows the 0x04 reply echoing 0x03. Accept both and
    // record which one actually arrived — bring-up step 8.
    bool cmdOk = (d[1] == _waitCmd) ||
                 (_waitCmd == (uint8_t)asmg::Cmd::ReadCurrentConfig &&
                  d[1] == (uint8_t)asmg::Cmd::SetCurrent);
    if (!addrOk || !cmdOk) return;

    uint32_t lat = micros() - _sentUs;
    _st.latLastUs = lat;
    if (lat > _st.latMaxUs) _st.latMaxUs = lat;
    _st.lastGoodMs = millis();

    switch (_waitCmd) {
        case (uint8_t)asmg::Cmd::ReadPosition:        // 0x02
            _st.position    = _unpack16(&d[2]);
            _st.cmdPosition = _unpack16(&d[4]);
            _st.havePos = _st.haveCmdPos = true;
            break;
        case (uint8_t)asmg::Cmd::ReadCurrentConfig:   // 0x04
            _st.torque         = _unpack16(&d[2]);
            _st.currentLimit   = _unpack16(&d[4]);
            _st.replyEchoCmd04 = d[1];
            _st.haveCfg = true;
            break;
        case (uint8_t)asmg::Cmd::ReadPid:             // 0x06
            _st.pidP = _unpack16(&d[2]);
            _st.pidI = _unpack16(&d[4]);
            _st.pidD = _unpack16(&d[6]);
            _st.havePid = true;
            break;
        case (uint8_t)asmg::Cmd::ReadPositionAndI:    // 0x07
            _st.position = _unpack16(&d[2]);
            _st.current  = _unpack16(&d[4]);
            _st.havePos = _st.haveCur = true;
            break;
        case (uint8_t)asmg::Cmd::ReadId:              // 0xFD
            _st.discoveredAddr = d[0];   // provisional — see parseReadId
            _st.haveAddr = true;
            break;
        default:
            break;
    }

    _waiting = false;
}

// ---------------------------------------------------------------------------
// Engine
// ---------------------------------------------------------------------------

void servo_can_poll() {
    if (!_ok) return;

    const uint32_t now = millis();

    static uint32_t lastHealthMs = 0;
    if (now - lastHealthMs >= 250) { lastHealthMs = now; _check_health(); }

    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(&_hfdcan, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&_hfdcan, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK) break;
        _trace_push(SERVO_TRACE_RX, hdr.Identifier, 8, data);
        _handle_rx(hdr.Identifier, data);
    }

    if (_waiting && (uint32_t)(micros() - _sentUs) >= REPLY_TIMEOUT_US) {
        _waiting = false;
        _st.timeouts++;
    }

    // A position command jumps the queue. 0x01 expects no reply, so it cannot
    // confuse the outstanding request, and making the clutch wait up to a
    // reply timeout for a telemetry frame would put a stall right back in the
    // shift path — the whole reason this node exists.
    if (_posPending && (now - _lastPosMs) >= POS_MIN_INTERVAL_MS) {
        _posPending = false;
        _lastPosMs  = now;
        asmg::Frame f = asmg::makeFrame(_addr, asmg::Cmd::SetPositionSpeed);
        uint16_t counts = _posCounts > asmg::kPositionMax ? asmg::kPositionMax : _posCounts;
        uint16_t speed  = _posSpeed  > asmg::kSpeedSlowest ? asmg::kSpeedSlowest : _posSpeed;
        _pack16(&f.data[2], counts);
        _pack16(&f.data[4], speed);
        _tx(f);
    }

    if (_waiting) return;    // strictly one request in flight

    uint8_t oneShot = 0;
    if      (_reqReadId)  { _reqReadId  = false; oneShot = (uint8_t)asmg::Cmd::ReadId; }
    else if (_reqReadPid) { _reqReadPid = false; oneShot = (uint8_t)asmg::Cmd::ReadPid; }
    else if (_reqReadCfg) { _reqReadCfg = false; oneShot = (uint8_t)asmg::Cmd::ReadCurrentConfig; }

    if (oneShot) {
        asmg::Frame f = (oneShot == (uint8_t)asmg::Cmd::ReadId)
            ? asmg::cmdReadId()
            : asmg::makeFrame(_addr, (asmg::Cmd)oneShot);
        if (_tx(f)) {
            _waiting  = true;
            _waitCmd  = oneShot;
            _waitAddr = _addr;
            _sentUs   = micros();
        }
        return;
    }

    if (_polling && (now - _lastPollMs) >= _pollMs) {
        _lastPollMs = now;
        asmg::Frame f = asmg::makeFrame(_addr, (asmg::Cmd)_pollCmd);
        if (_tx(f)) {
            _waiting  = true;
            _waitCmd  = _pollCmd;
            _waitAddr = _addr;
            _sentUs   = micros();
        }
    }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

const ServoStatus *servo_status() {
    _st.replyFresh = _st.lastGoodMs != 0 && (millis() - _st.lastGoodMs) < SERVO_STALE_MS;
    return &_st;
}

bool     servo_link_ok()  { return servo_status()->replyFresh; }
uint16_t servo_position() { return _st.position; }

void servo_command_position(uint16_t counts, uint16_t speed) {
    _posCounts  = counts;
    _posSpeed   = speed;
    _posPending = true;      // mailbox, not a queue: the newest target wins
}

void servo_request_read_id()  { _reqReadId  = true; }
void servo_request_read_pid() { _reqReadPid = true; }
void servo_request_read_cfg() { _reqReadCfg = true; }

void servo_clear_counters() {
    _st.timeouts = _st.txFails = 0;
    _st.rxCount  = _st.txCount = 0;
    _st.latLastUs = _st.latMaxUs = 0;
}

void     servo_set_address(uint8_t addr)   { _addr = addr; }
uint8_t  servo_address()                   { return _addr; }
void     servo_set_big_endian(bool be)     { _bigEndian = be; }
bool     servo_big_endian()                { return _bigEndian; }
void     servo_set_polling(bool on)        { _polling = on; }
bool     servo_polling()                   { return _polling; }
void     servo_set_poll_cmd(uint8_t cmd)   { _pollCmd = cmd; }
uint8_t  servo_poll_cmd()                  { return _pollCmd; }
uint16_t servo_poll_interval_ms()          { return _pollMs; }

void servo_set_poll_interval_ms(uint16_t ms) {
    // A period shorter than the reply deadline would start a poll while the
    // last one is still outstanding, which this engine will not do — it would
    // just spin. Floor it above the timeout.
    _pollMs = ms < (REPLY_TIMEOUT_US / 1000) ? (REPLY_TIMEOUT_US / 1000) : ms;
}

uint32_t servo_trace_seq() { return _traceSeq; }

uint32_t servo_trace_read(uint32_t *lastSeq, ServoTrace *out, uint32_t maxOut) {
    uint32_t newest = _traceSeq;
    uint32_t from   = *lastSeq;
    // A reader more than a ring behind has already lost those frames; skip it
    // forward rather than hand it entries that have been overwritten.
    if ((uint32_t)(newest - from) > TRACE_LEN) from = newest - TRACE_LEN;

    uint32_t n = 0;
    while (from != newest && n < maxOut) out[n++] = _trace[from++ & (TRACE_LEN - 1)];
    *lastSeq = from;
    return n;
}
