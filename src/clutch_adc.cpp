#include <Arduino.h>
#include "clutch_adc.h"
#include "pins.h"

// VREF+ on the WeAct board. If a precision reference is fitted later, only this
// changes — the clutch thresholds are captured live in post-divider volts, so
// nothing downstream cares about absolute scale.
#define VREF_VOLTS      3.3f

// 16-bit differential: INP == INN reads mid-scale, full negative is 0.
#define ADC_FULL_SCALE  65535.0f
#define ADC_MID_SCALE   32768

static ADC_HandleTypeDef _hadc1;   // clutch feedback, differential, free-running
static ADC_HandleTypeDef _hadc2;   // paddle halls, single-ended, on demand
static bool _ok = false;

static bool _init_clutch() {
    GPIO_InitTypeDef g = {};
    g.Pin  = GPIO_PIN_0 | GPIO_PIN_1;      // PC0 = INP, PC1 = INN
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &g);

    _hadc1.Instance                      = ADC1;
    _hadc1.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
    _hadc1.Init.Resolution               = ADC_RESOLUTION_16B;
    _hadc1.Init.ScanConvMode             = ADC_SCAN_DISABLE;
    _hadc1.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    _hadc1.Init.LowPowerAutoWait         = DISABLE;
    _hadc1.Init.ContinuousConvMode       = ENABLE;    // free-running: a read is never a wait
    _hadc1.Init.NbrOfConversion          = 1;
    _hadc1.Init.DiscontinuousConvMode    = DISABLE;
    _hadc1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    _hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    _hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    _hadc1.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;   // always the newest
    _hadc1.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
    // 64x oversampling in hardware, shifted back to 16 bits. This is what
    // replaces the ADS1115's steadiness, at zero CPU cost.
    _hadc1.Init.OversamplingMode                  = ENABLE;
    _hadc1.Init.Oversampling.Ratio                = 64;
    _hadc1.Init.Oversampling.RightBitShift        = ADC_RIGHTBITSHIFT_6;
    _hadc1.Init.Oversampling.TriggeredMode        = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    _hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
    if (HAL_ADC_Init(&_hadc1) != HAL_OK) return false;

    // Calibration MUST be run in differential mode to null the differential
    // offset; calibrating single-ended and then switching leaves a bias.
    if (HAL_ADCEx_Calibration_Start(&_hadc1, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
        return false;

    ADC_ChannelConfTypeDef ch = {};
    ch.Channel      = ADC_CH_CLUTCH;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;   // long sample: the divider is a high source impedance
    ch.SingleDiff   = ADC_DIFFERENTIAL_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&_hadc1, &ch) != HAL_OK) return false;

    return HAL_ADC_Start(&_hadc1) == HAL_OK;
}

static bool _init_halls() {
    GPIO_InitTypeDef g = {};
    g.Pin  = GPIO_PIN_2 | GPIO_PIN_3;      // PA2, PA3
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);

    _hadc2.Instance                      = ADC2;
    _hadc2.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
    _hadc2.Init.Resolution               = ADC_RESOLUTION_16B;
    _hadc2.Init.ScanConvMode             = ADC_SCAN_DISABLE;
    _hadc2.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    _hadc2.Init.LowPowerAutoWait         = DISABLE;
    _hadc2.Init.ContinuousConvMode       = DISABLE;   // one channel at a time, on demand
    _hadc2.Init.NbrOfConversion          = 1;
    _hadc2.Init.DiscontinuousConvMode    = DISABLE;
    _hadc2.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    _hadc2.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    _hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    _hadc2.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    _hadc2.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
    _hadc2.Init.OversamplingMode         = DISABLE;
    if (HAL_ADC_Init(&_hadc2) != HAL_OK) return false;

    return HAL_ADCEx_Calibration_Start(&_hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) == HAL_OK;
}

bool clutch_adc_init() {
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    _ok = _init_clutch() && _init_halls();
    return _ok;
}

int32_t clutch_adc_raw() {
    if (!_ok) return 0;
    // Free-running with overwrite-on-overrun, so the data register always holds
    // the most recent completed conversion. No wait, no failure mode.
    return (int32_t)HAL_ADC_GetValue(&_hadc1) - ADC_MID_SCALE;
}

float clutch_adc_volts() {
    return (float)clutch_adc_raw() * (VREF_VOLTS / ADC_FULL_SCALE);
}

// One-shot single-ended read. Bounded at a few microseconds — this is a
// peripheral inside the chip, not a bus that another device can stall.
static uint16_t _hall_read(uint32_t channel) {
    if (!_ok) return 0;

    ADC_ChannelConfTypeDef ch = {};
    ch.Channel      = channel;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
    ch.SingleDiff   = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&_hadc2, &ch) != HAL_OK) return 0;

    if (HAL_ADC_Start(&_hadc2) != HAL_OK) return 0;
    uint16_t v = 0;
    if (HAL_ADC_PollForConversion(&_hadc2, 2) == HAL_OK) v = (uint16_t)HAL_ADC_GetValue(&_hadc2);
    HAL_ADC_Stop(&_hadc2);
    return v;
}

uint16_t hall_read_left()  { return _hall_read(ADC_CH_HALL_LEFT); }
uint16_t hall_read_right() { return _hall_read(ADC_CH_HALL_RIGHT); }
