#include <Arduino.h>
#include <string.h>
#include "can_main.h"
#include "can_ids.h"

#define MAIN_NODE_FW_VERSION  0x00D6   // 214 — continues the ESP32 main node's numbering
#define GEAR_TIMEOUT_MS       1000
#define RPM_TX_PERIOD_MS      20       // 50 Hz, as the ESP32 node broadcast

static FDCAN_HandleTypeDef _hfdcan;
static bool     _ok = false;
static bool     _busLive = false;
static uint8_t  _gear = GEAR_UNKNOWN;
static uint32_t _lastGearMs = 0;
static uint32_t _rxCount = 0;
static uint8_t  _txSeq = 0;

bool can_init() {
    // FDCAN kernel clock from PLL1Q. Generic H723 variant clocks:
    // HSI 64 / M4 * N34 = VCO 544 MHz, /Q4 -> PLL1Q = 136 MHz.
    RCC_PeriphCLKInitTypeDef pclk = {};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    pclk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) return false;

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef g = {};
    g.Pin       = GPIO_PIN_0 | GPIO_PIN_1;   // PD0 = FDCAN1_RX, PD1 = FDCAN1_TX
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &g);

    _hfdcan.Instance                  = FDCAN1;
    _hfdcan.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    _hfdcan.Init.Mode                 = FDCAN_MODE_NORMAL;
    _hfdcan.Init.AutoRetransmission   = ENABLE;
    _hfdcan.Init.TransmitPause        = DISABLE;
    _hfdcan.Init.ProtocolException    = DISABLE;
    // 136 MHz / prescaler 8 = 17 MHz; 34 tq/bit (1 + 26 + 7) = 500 kbit/s,
    // sample point 79.4 %
    _hfdcan.Init.NominalPrescaler     = 8;
    _hfdcan.Init.NominalSyncJumpWidth = 7;
    _hfdcan.Init.NominalTimeSeg1      = 26;
    _hfdcan.Init.NominalTimeSeg2      = 7;
    _hfdcan.Init.DataPrescaler        = 8;
    _hfdcan.Init.DataSyncJumpWidth    = 7;
    _hfdcan.Init.DataTimeSeg1         = 26;
    _hfdcan.Init.DataTimeSeg2         = 7;
    _hfdcan.Init.MessageRAMOffset     = 0;
    _hfdcan.Init.StdFiltersNbr        = 0;
    _hfdcan.Init.ExtFiltersNbr        = 2;
    _hfdcan.Init.RxFifo0ElmtsNbr      = 32;
    _hfdcan.Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_8;
    _hfdcan.Init.RxFifo1ElmtsNbr      = 0;
    _hfdcan.Init.RxFifo1ElmtSize      = FDCAN_DATA_BYTES_8;
    _hfdcan.Init.RxBuffersNbr         = 0;
    _hfdcan.Init.RxBufferSize         = FDCAN_DATA_BYTES_8;
    _hfdcan.Init.TxEventsNbr          = 0;
    _hfdcan.Init.TxBuffersNbr         = 0;
    _hfdcan.Init.TxFifoQueueElmtsNbr  = 8;
    _hfdcan.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
    _hfdcan.Init.TxElmtSize           = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(&_hfdcan) != HAL_OK) return false;

    // Exact-match on FULL 29-bit IDs, never on the msgtype nibble — msgtypes
    // collide across nodes.
    static const uint32_t rxIds[2] = { CAN_REAR_GEAR_POS, CAN_HB_REAR };
    for (uint32_t i = 0; i < 2; i++) {
        FDCAN_FilterTypeDef f = {};
        f.IdType       = FDCAN_EXTENDED_ID;
        f.FilterIndex  = i;
        f.FilterType   = FDCAN_FILTER_DUAL;
        f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        f.FilterID1    = rxIds[i];
        f.FilterID2    = rxIds[i];
        if (HAL_FDCAN_ConfigFilter(&_hfdcan, &f) != HAL_OK) return false;
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&_hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
        return false;

    if (HAL_FDCAN_Start(&_hfdcan) != HAL_OK) return false;

    _ok = true;
    return true;
}

// Queue one frame. data[0]=seq, data[1]=status, data[2..7]=payload.
static void _tx(uint32_t id, const uint8_t *payload6) {
    if (!_ok) return;

    uint8_t data[8] = { _txSeq++, NODE_STATUS_OK };
    memcpy(&data[2], payload6, 6);

    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_EXTENDED_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    HAL_FDCAN_AddMessageToTxFifoQ(&_hfdcan, &tx, data);
}

void can_poll() {
    if (!_ok) return;

    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(&_hfdcan, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&_hfdcan, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK) break;

        _rxCount++;
        _busLive = true;    // heard a frame -> the bus is up and ACKing, TX is safe

        if (hdr.Identifier == CAN_REAR_GEAR_POS) {
            _gear       = data[2];
            _lastGearMs = millis();
        }
    }
}

void can_tick(uint16_t rpm, bool manualMode) {
    if (!_ok || !_busLive) return;   // never flood a bus that is not yet ACKing

    uint32_t now = millis();

    static uint32_t lastRpmMs = 0;
    if (now - lastRpmMs >= RPM_TX_PERIOD_MS) {
        lastRpmMs = now;
        uint8_t p[6] = { (uint8_t)(rpm & 0xFF), (uint8_t)(rpm >> 8) };
        _tx(CAN_MAIN_RPM, p);
    }

    static uint32_t lastHbMs = 0;
    if (now - lastHbMs >= CAN_HEARTBEAT_INTERVAL_MS) {
        lastHbMs = now;
        uint8_t p[6] = { (uint8_t)(MAIN_NODE_FW_VERSION & 0xFF),
                         (uint8_t)(MAIN_NODE_FW_VERSION >> 8) };
        _tx(CAN_HB_MAIN, p);

        uint8_t s[6] = { (uint8_t)(manualMode ? 1 : 0) };
        _tx(CAN_MAIN_SHIFT_STATUS, s);
    }
}

void can_send_shift_up(uint16_t shiftMs, uint16_t ignCutMs, uint8_t targetGear) {
    uint8_t p[6] = { (uint8_t)(shiftMs & 0xFF),  (uint8_t)(shiftMs >> 8),
                     (uint8_t)(ignCutMs & 0xFF), (uint8_t)(ignCutMs >> 8),
                     targetGear, 0 };
    _tx(CAN_REAR_CMD_SHIFT_UP, p);
}

void can_send_shift_down(uint16_t shiftMs, uint8_t targetGear) {
    uint8_t p[6] = { (uint8_t)(shiftMs & 0xFF), (uint8_t)(shiftMs >> 8),
                     0, 0, targetGear, 0 };
    _tx(CAN_REAR_CMD_SHIFT_DN, p);
}

void can_send_shift_stack(uint8_t targetGear) {
    uint8_t p[6] = { targetGear };
    _tx(CAN_MAIN_SHIFT_STACK, p);
}

uint8_t  can_gear()        { return _gear; }
uint32_t can_rx_count()    { return _rxCount; }
bool     can_bus_live()    { return _busLive; }
uint32_t can_gear_age_ms() { return _lastGearMs ? (millis() - _lastGearMs) : 0xFFFFFFFF; }

bool can_gear_valid() {
    return _lastGearMs != 0 && (millis() - _lastGearMs) < GEAR_TIMEOUT_MS
           && _gear != GEAR_UNKNOWN;
}
