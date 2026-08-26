#include <Arduino.h>
#include "hall_adc.h"
#include "pins.h"

static ADC_HandleTypeDef _hadc2;   // paddle halls, single-ended, on demand
static bool _ok = false;

bool hall_adc_init() {
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

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

    _ok = HAL_ADCEx_Calibration_Start(&_hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) == HAL_OK;
    return _ok;
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
