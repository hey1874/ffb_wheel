/*
 * buttons.c - MCP23017 I2C button scanner (debounced)
 *
 * Each chip: both ports as inputs with pull-ups; read GPIOA+GPIOB (16 bits) and
 * invert (active-low → pressed=1). Per-button debounce accepts a state change
 * only after BUTTON_DEBOUNCE agreeing samples. All I2C uses timeouts so a
 * missing chip never stalls the caller (core0 / USB loop).
 */
#include "buttons.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <stdbool.h>

#if BUTTON_CHIPS > 0

/* MCP23017 registers (IOCON.BANK=0, power-on default). */
#define MCP_IODIRA  0x00   /* +1 = IODIRB */
#define MCP_GPPUA   0x0C   /* +1 = GPPUB  */
#define MCP_GPIOA   0x12   /* +1 = GPIOB, sequential read auto-increments */

#define BTN_BITS       (16 * BUTTON_CHIPS)
/* Short timeout: a present chip answers in tens of µs; this only bounds the
 * cost of a stuck bus. Kept small so an unresponsive chip can't add more than
 * ~0.3 ms per transaction to the core0 input-report pass. */
#define I2C_TIMEOUT_US 300

static const uint8_t s_addr[BUTTON_CHIPS] = {
    BUTTON_ADDR0,
#if BUTTON_CHIPS > 1
    BUTTON_ADDR1,
#endif
};

static uint8_t  s_cnt[BTN_BITS];   /* per-button consecutive-disagreement count */
static uint32_t s_state;           /* debounced bitmask */

_Static_assert(BUTTON_CHIPS <= 2, "BUTTON_CHIPS max 2 (HID button field is 32 bits)");

static bool mcp_write(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t b[2] = { reg, val };
    return i2c_write_timeout_us(BUTTON_I2C, addr, b, 2, false, I2C_TIMEOUT_US) == 2;
}

/* Read 16 input bits from one chip (pressed = 1). Leaves *out untouched on
 * error so a dropped read holds the last debounced state. */
static bool mcp_read16(uint8_t addr, uint16_t *out) {
    uint8_t reg = MCP_GPIOA;
    if (i2c_write_timeout_us(BUTTON_I2C, addr, &reg, 1, true, I2C_TIMEOUT_US) != 1)
        return false;
    uint8_t d[2];
    if (i2c_read_timeout_us(BUTTON_I2C, addr, d, 2, false, I2C_TIMEOUT_US) != 2)
        return false;
    *out = (uint16_t)~(((uint16_t)d[1] << 8) | d[0]);  /* active-low → pressed=1 */
    return true;
}

#endif /* BUTTON_CHIPS > 0 */

void buttons_init(void) {
#if BUTTON_CHIPS > 0
    i2c_init(BUTTON_I2C, BUTTON_I2C_HZ);
    gpio_set_function(BUTTON_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BUTTON_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BUTTON_SDA_PIN);
    gpio_pull_up(BUTTON_SCL_PIN);

    for (int c = 0; c < BUTTON_CHIPS; c++) {
        mcp_write(s_addr[c], MCP_IODIRA,     0xFF);  /* PORTA inputs */
        mcp_write(s_addr[c], MCP_IODIRA + 1, 0xFF);  /* PORTB inputs */
        mcp_write(s_addr[c], MCP_GPPUA,      0xFF);  /* PORTA pull-ups on */
        mcp_write(s_addr[c], MCP_GPPUA + 1,  0xFF);  /* PORTB pull-ups on */
    }
    s_state = 0;
    for (int i = 0; i < BTN_BITS; i++) s_cnt[i] = 0;
#endif
}

uint32_t buttons_read(void) {
#if BUTTON_CHIPS > 0
    uint32_t raw = s_state;   /* per-chip read failure holds that chip's bits */
    for (int c = 0; c < BUTTON_CHIPS; c++) {
        uint16_t v;
        if (mcp_read16(s_addr[c], &v))
            raw = (raw & ~(0xFFFFu << (16 * c))) | ((uint32_t)v << (16 * c));
    }
    for (int i = 0; i < BTN_BITS; i++) {
        bool rb  = (raw >> i) & 1u;
        bool cur = (s_state >> i) & 1u;
        if (rb != cur) {
            if (++s_cnt[i] >= BUTTON_DEBOUNCE) {
                s_state ^= (1u << i);
                s_cnt[i] = 0;
            }
        } else {
            s_cnt[i] = 0;
        }
    }
    return s_state;
#else
    return 0;
#endif
}
