/*
 * mcp2515.c - MCP2515 SPI-CAN driver for RP2040 (Pico-SDK)
 *
 * Polling-mode driver: enough for ~1 kHz FFB torque + encoder polling.
 * TX uses buffer 0 (One-Shot Mode, no retransmission); RX uses buffer 0
 * with rollover into buffer 1, both drained by mcp2515_receive().
 */
#include "mcp2515.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/time.h"

/* ---- SPI helpers ---- */
static void cs_low(void)  { gpio_put(MCP_CS_PIN, 0); }
static void cs_high(void) { gpio_put(MCP_CS_PIN, 1); }

static void mcp_write_reg(uint8_t addr, uint8_t val) {
    uint8_t cmd[3] = {MCP_WRITE, addr, val};
    cs_low();
    spi_write_blocking(MCP_SPI, cmd, 3);
    cs_high();
}

static uint8_t mcp_read_reg(uint8_t addr) {
    uint8_t cmd[2] = {MCP_READ, addr};
    uint8_t val = 0;
    cs_low();
    spi_write_blocking(MCP_SPI, cmd, 2);
    spi_read_blocking(MCP_SPI, 0, &val, 1);
    cs_high();
    return val;
}

static void mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t val) {
    uint8_t cmd[4] = {MCP_BIT_MODIFY, addr, mask, val};
    cs_low();
    spi_write_blocking(MCP_SPI, cmd, 4);
    cs_high();
}

uint8_t mcp2515_read_reg(uint8_t addr) { return mcp_read_reg(addr); }

/* ---- Bitrate config (common crystal × baud combos) ---- */
static void set_bitrate(void) {
    uint8_t cnf1, cnf2, cnf3;
#if MCP_OSC_HZ == 8000000UL && MCP_BAUD == 500000UL
    cnf1 = 0x00; cnf2 = 0x90; cnf3 = 0x02;   /* 8 TQ, 87.5% sample point */
#elif MCP_OSC_HZ == 8000000UL && MCP_BAUD == 250000UL
    cnf1 = 0x00; cnf2 = 0xB1; cnf3 = 0x05;   /* 16 TQ */
#elif MCP_OSC_HZ == 16000000UL && MCP_BAUD == 500000UL
    cnf1 = 0x00; cnf2 = 0xF0; cnf3 = 0x86;   /* 16 TQ */
#elif MCP_OSC_HZ == 16000000UL && MCP_BAUD == 1000000UL
    cnf1 = 0x00; cnf2 = 0x80; cnf3 = 0x80;   /* 8 TQ */
#else
    /* Fallback: 8 MHz / 500 k. Edit for your combo. */
    cnf1 = 0x00; cnf2 = 0x90; cnf3 = 0x02;
#endif
    mcp_write_reg(MCP_CNF1, cnf1);
    mcp_write_reg(MCP_CNF2, cnf2);
    mcp_write_reg(MCP_CNF3, cnf3);
}

/* ---- Public API ---- */
bool mcp2515_init(void) {
    /* SPI + GPIO setup */
    spi_init(MCP_SPI, MCP_SPI_HZ);
    gpio_set_function(MCP_SCK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(MCP_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MCP_MISO_PIN, GPIO_FUNC_SPI);

    gpio_init(MCP_CS_PIN);
    gpio_set_dir(MCP_CS_PIN, GPIO_OUT);
    cs_high();

    /* Reset MCP2515 (wait for oscillator to start) */
    uint8_t rst = MCP_RESET;
    cs_low();
    spi_write_blocking(MCP_SPI, &rst, 1);
    cs_high();
    sleep_ms(10);

    /* Enter CONFIG mode */
    mcp_write_reg(MCP_CANCTRL, MCP_MODE_CONFIG);
    for (int i = 0; i < 20; i++) {
        if ((mcp_read_reg(MCP_CANSTAT) & 0xE0) == MCP_MODE_CONFIG) break;
        sleep_ms(2);
    }
    if ((mcp_read_reg(MCP_CANSTAT) & 0xE0) != MCP_MODE_CONFIG) return false;

    set_bitrate();

    /* RXB0: receive all standard frames (RXM=11), no filters (mask off),
     * with buffer rollover to RXB1 (BUKT). RXB1 likewise accepts all frames
     * and is drained by mcp2515_receive(). */
    mcp_write_reg(MCP_RXB0CTRL, 0x64);   /* RXM[1:0]=11, BUKT=1 */
    mcp_write_reg(MCP_RXB1CTRL, 0x60);   /* RXM[1:0]=11 */

    /* Clear interrupts / flags */
    mcp_write_reg(MCP_CANINTF, 0x00);
    mcp_write_reg(MCP_CANINTE, 0x00);    /* polling: no interrupts */

    /* Enter NORMAL mode with One-Shot TX (see MCP_OSM in the header). */
    mcp_write_reg(MCP_CANCTRL, MCP_MODE_NORMAL | MCP_OSM);
    for (int i = 0; i < 20; i++) {
        if ((mcp_read_reg(MCP_CANSTAT) & 0xE0) == MCP_MODE_NORMAL) break;
        sleep_ms(2);
    }
    return (mcp_read_reg(MCP_CANSTAT) & 0xE0) == MCP_MODE_NORMAL;
}

bool mcp2515_send(uint16_t id, const uint8_t *data, uint8_t len) {
    if (len > 8) len = 8;
    /* Non-blocking. If TXB0 is still occupied by a previous frame (lost
     * arbitration, bus error, no ACK), abort it: for a 1 kHz torque stream the
     * newest command supersedes a stale one, and never busy-waiting keeps this
     * off any hot path. In One-Shot Mode TXREQ self-clears after one attempt,
     * so this abort only fires on a genuinely stuck/in-flight frame. */
    if (mcp_read_reg(MCP_TXB0CTRL) & 0x08)
        mcp_bit_modify(MCP_TXB0CTRL, 0x08, 0x00);   /* clear TXREQ = abort */

    /* Load the whole frame (SIDH..DLC + data) in ONE CS assertion via the
     * LOAD TX BUFFER instruction, instead of ~13 discrete register writes. */
    uint8_t frame[6 + 8];
    frame[0] = MCP_LOAD_TX0;                 /* load starting @ TXB0SIDH */
    frame[1] = (uint8_t)(id >> 3);           /* SIDH: SID[10:3] */
    frame[2] = (uint8_t)((id & 0x07) << 5);  /* SIDL: SID[2:0], EXIDE=0 (std) */
    frame[3] = 0;                            /* EID8 (unused, standard frame) */
    frame[4] = 0;                            /* EID0 (unused) */
    frame[5] = (uint8_t)(len & 0x0F);        /* DLC, RTR=0 */
    for (uint8_t i = 0; i < len; i++)
        frame[6 + i] = data[i];

    cs_low();
    spi_write_blocking(MCP_SPI, frame, (size_t)(6 + len));
    cs_high();

    /* Request-to-send TXB0 (RTS is its own single-byte instruction). */
    uint8_t rts = MCP_RTS_TX0;
    cs_low();
    spi_write_blocking(MCP_SPI, &rts, 1);
    cs_high();
    return true;
}

bool mcp2515_receive(uint16_t *id, uint8_t *data, uint8_t *len) {
    uint8_t status_cmd = MCP_READ_STATUS;
    uint8_t status = 0;
    cs_low();
    spi_write_blocking(MCP_SPI, &status_cmd, 1);
    spi_read_blocking(MCP_SPI, 0, &status, 1);
    cs_high();

    /* Drain RXB0 first, then RXB1 (rollover target). Skipping RXB1 would
     * leave it permanently full after the first rollover. */
    uint8_t cmd;
    if (status & 0x01)        cmd = MCP_READ_RX0;   /* RX0IF */
    else if (status & 0x02)   cmd = MCP_READ_RX1;   /* RX1IF */
    else                      return false;

    /* One CS assertion reads SIDH,SIDL,EID8,EID0,DLC + 8 data bytes. Raising CS
     * after a READ RX BUFFER instruction auto-clears the RXnIF flag, so no
     * separate CANINTF bit-modify is needed. */
    uint8_t buf[13];
    cs_low();
    spi_write_blocking(MCP_SPI, &cmd, 1);
    spi_read_blocking(MCP_SPI, 0, buf, sizeof(buf));
    cs_high();

    *id = ((uint16_t)buf[0] << 3) | ((buf[1] >> 5) & 0x07);
    uint8_t l = buf[4] & 0x0F;
    if (l > 8) l = 8;
    *len = l;
    for (uint8_t i = 0; i < l; i++)
        data[i] = buf[5 + i];
    return true;
}
