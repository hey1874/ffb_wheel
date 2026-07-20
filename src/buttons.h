/*
 * buttons.h - Momentary buttons via MCP23017 I2C GPIO expander(s)
 *
 * One MCP23017 gives 16 buttons over just 2 wires (SDA/SCL); a second chip
 * (different address) brings it to the full 32 the HID descriptor exposes.
 * Buttons short their expander pin to GND when pressed; the MCP23017's internal
 * pull-ups are enabled, so unwired pins read "not pressed".
 *
 * WIRING: button between an MCP23017 GPIO (GPA0..7 / GPB0..7) and GND. Tie the
 * chip's A2/A1/A0 address pins to set 0x20..0x27. SDA/SCL to the RP2040 I2C
 * pins below (each needs a pull-up to 3V3 — the on-chip pulls are weak, add
 * ~4.7k externally for reliability). Do NOT power the chip from 5V if its I2C
 * lines reach the RP2040 (not 5V tolerant).
 *
 * All I2C access uses timeouts, so a missing/unresponsive chip degrades to
 * "no buttons" instead of stalling the USB loop on core0.
 */
#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdint.h>

/* Number of MCP23017 chips: 0 = disabled, 1 = 16 buttons, 2 = 32 buttons. */
#ifndef BUTTON_CHIPS
#define BUTTON_CHIPS 0
#endif

/* RP2040 I2C instance and pins. Defaults: i2c0 on GP20 (SDA) / GP21 (SCL),
 * which avoid SPI0 (GP16-19), the LED (GP25) and the ADC pedal pins. */
#ifndef BUTTON_I2C
#define BUTTON_I2C i2c0
#endif
#ifndef BUTTON_SDA_PIN
#define BUTTON_SDA_PIN 20
#endif
#ifndef BUTTON_SCL_PIN
#define BUTTON_SCL_PIN 21
#endif
#ifndef BUTTON_I2C_HZ
#define BUTTON_I2C_HZ 400000
#endif

/* 7-bit addresses of chip 0 (buttons 1-16) and chip 1 (buttons 17-32),
 * set by each chip's A2/A1/A0 pins (0x20..0x27). */
#ifndef BUTTON_ADDR0
#define BUTTON_ADDR0 0x20
#endif
#ifndef BUTTON_ADDR1
#define BUTTON_ADDR1 0x21
#endif

/* Consecutive agreeing samples required to accept a change (debounce). Scanned
 * at the input-report rate (~250 Hz), so 3 ≈ 12 ms. */
#ifndef BUTTON_DEBOUNCE
#define BUTTON_DEBOUNCE 3
#endif

/* Configure I2C and the expander(s). Call once at startup. */
void buttons_init(void);

/* Debounced button state: bit i set = button (i+1) pressed. */
uint32_t buttons_read(void);

#endif /* BUTTONS_H */
