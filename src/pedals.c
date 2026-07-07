/*
 * pedals.c - Analog pedal inputs via the RP2040/RP2350 on-chip ADC
 *
 * Per pedal: oversample the ADC to knock down noise, run an exponential
 * lowpass, autorange the observed travel, and map to a signed HID axis.
 * The ADC is 12-bit (0..4095) on both chips and the SDK API is identical,
 * so this file is target-agnostic.
 */
#include "pedals.h"
#include "hardware/adc.h"

#define ADC_FULL_SCALE   4095       /* 12-bit ADC */
#define OVERSAMPLE       8          /* reads averaged per pedal per call */
#define EMA_SHIFT        2          /* lowpass: filt += (raw-filt) >> SHIFT */
#define MIN_TRAVEL       64         /* raw span below this = "not calibrated" */

static struct {
    uint16_t rmin;      /* smallest filtered raw seen (autorange low)  */
    uint16_t rmax;      /* largest filtered raw seen  (autorange high) */
    int32_t  filt;      /* lowpass state (raw units)                   */
    bool     seeded;    /* filt primed with the first sample?          */
} s_ped[PEDAL_COUNT];

void pedals_init(void) {
    adc_init();
    for (int i = 0; i < PEDAL_COUNT; i++) {
        adc_gpio_init(PEDAL_ADC_BASE_GPIO + i);
        /* Seed the autorange inverted so the first samples expand it. */
        s_ped[i].rmin   = ADC_FULL_SCALE;
        s_ped[i].rmax   = 0;
        s_ped[i].filt   = 0;
        s_ped[i].seeded = false;
    }
}

static uint16_t read_raw_avg(uint8_t ch) {
    adc_select_input(ch);
    uint32_t acc = 0;
    for (int i = 0; i < OVERSAMPLE; i++)
        acc += adc_read();
    return (uint16_t)(acc / OVERSAMPLE);
}

void pedals_read(int16_t *out) {
    for (int i = 0; i < PEDAL_COUNT; i++) {
        uint16_t raw = read_raw_avg(i);

        /* Exponential lowpass. */
        if (!s_ped[i].seeded) {
            s_ped[i].filt   = raw;
            s_ped[i].seeded = true;
        } else {
            s_ped[i].filt += ((int32_t)raw - s_ped[i].filt) >> EMA_SHIFT;
        }
        uint16_t f = (uint16_t)s_ped[i].filt;

        /* Autorange: widen the calibrated span as the pedal is exercised. */
        if (f < s_ped[i].rmin) s_ped[i].rmin = f;
        if (f > s_ped[i].rmax) s_ped[i].rmax = f;

        int32_t span = (int32_t)s_ped[i].rmax - s_ped[i].rmin;
        int32_t unit;   /* 0..32767: 0 = released, 32767 = fully pressed */
        if (span < MIN_TRAVEL) {
            unit = 0;   /* not enough travel observed yet: hold at released */
        } else {
            unit = (int32_t)(f - s_ped[i].rmin) * 32767 / span;
            if (unit < 0)     unit = 0;
            if (unit > 32767) unit = 32767;
        }

        if (PEDAL_INVERT_MASK & (1u << i))
            unit = 32767 - unit;

        /* Map unipolar 0..32767 to the full signed axis (released = -32767). */
        out[i] = (int16_t)(unit * 2 - 32767);
    }
}
