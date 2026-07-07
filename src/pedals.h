/*
 * pedals.h - Analog pedal inputs (throttle / brake / clutch) via RP2040/RP2350 ADC
 *
 * Reads potentiometer pedals on the on-chip 12-bit ADC and maps each to a
 * signed HID axis (-32767..32767), released at the negative end. Uses
 * oversampling + a lowpass filter for noise and OpenFFBoard-style autoranging
 * so the travel calibrates itself as each pedal is pressed through its range.
 *
 * WIRING (IMPORTANT — the ADC pins are NOT 5V tolerant on either chip):
 *   - Passive potentiometer pedals: wire the pot across 3V3(OUT) and GND,
 *     wiper to the ADC GPIO. Do NOT power the pot from 5V.
 *   - Active 0..5V sensors: scale to 0..3.3V (divider) or use an external
 *     ADC (e.g. ADS1115) — do not connect 5V to these pins.
 *
 * Default channels: ADC0/1/2 = GP26/GP27/GP28 (throttle, brake, clutch).
 * Leave a pedal unwired only if you also lower PEDAL_COUNT — a floating ADC
 * pin autoranges on noise and produces garbage.
 */
#ifndef PEDALS_H
#define PEDALS_H

#include <stdint.h>

/* Number of analog pedals (consecutive ADC channels starting at channel 0). */
#ifndef PEDAL_COUNT
#define PEDAL_COUNT 3
#endif

/* GPIO of ADC channel 0. On both RP2040 and RP2350 this is GP26; channels
 * 1/2 are GP27/GP28. Change only if your board wires pedals elsewhere. */
#ifndef PEDAL_ADC_BASE_GPIO
#define PEDAL_ADC_BASE_GPIO 26
#endif

/* Bit i set = invert pedal i (use when pressing DECREASES the wiper voltage,
 * so that "released" still maps to the negative axis end). */
#ifndef PEDAL_INVERT_MASK
#define PEDAL_INVERT_MASK 0x00
#endif

/* Configure the ADC and pedal GPIOs. Call once at startup. */
void pedals_init(void);

/* Sample all pedals and write PEDAL_COUNT calibrated axis values (signed,
 * -32767..32767; released = -32767) into out[], in channel order. */
void pedals_read(int16_t *out);

#endif /* PEDALS_H */
