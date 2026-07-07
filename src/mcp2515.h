/*
 * mcp2515.h - MCP2515 CAN controller SPI driver (RP2040)
 *
 * Wiring defaults (override via -D at build time):
 *   SPI  = spi0
 *   SCK  = GP18   MOSI = GP19   MISO = GP16   CS = GP17
 *   OSC  = 8 MHz  (MCP2515 module crystal)
 *   BAUD = 500000 (match ODrive CAN config)
 *
 * Polling mode (no INT pin). Good enough for ~1 kHz FFB torque + encoder
 * polling. If you need lower latency, wire INT to a GPIO and switch to
 * interrupt-driven receive.
 */
#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"

#ifndef MCP_SPI
#define MCP_SPI     spi0
#endif
#ifndef MCP_SCK_PIN
#define MCP_SCK_PIN 18
#endif
#ifndef MCP_MOSI_PIN
#define MCP_MOSI_PIN 19
#endif
#ifndef MCP_MISO_PIN
#define MCP_MISO_PIN 16
#endif
#ifndef MCP_CS_PIN
#define MCP_CS_PIN  17
#endif
#ifndef MCP_OSC_HZ
#define MCP_OSC_HZ  8000000UL
#endif
#ifndef MCP_BAUD
#define MCP_BAUD    500000UL
#endif
/* SPI clock to the MCP2515. Max 10 MHz per datasheet; 5 MHz is safe. */
#ifndef MCP_SPI_HZ
#define MCP_SPI_HZ  5000000UL
#endif

/* MCP2515 SPI instructions */
#define MCP_RESET        0xC0
#define MCP_READ         0x03
#define MCP_WRITE        0x02
#define MCP_BIT_MODIFY   0x05
#define MCP_READ_STATUS  0xA0
#define MCP_LOAD_TX0     0x40
#define MCP_RTS_TX0      0x81
#define MCP_READ_RX0     0x90

/* Key registers */
#define MCP_CANCTRL      0x0F
#define MCP_CANSTAT      0x0E
#define MCP_CNF1         0x2A
#define MCP_CNF2         0x29
#define MCP_CNF3         0x28
#define MCP_CANINTF      0x2C
#define MCP_CANINTE      0x2B
#define MCP_RXB0CTRL     0x60
#define MCP_TXB0CTRL     0x30
#define MCP_TXB0SIDH     0x31
#define MCP_TXB0SIDL     0x32
#define MCP_TXB0DLC      0x35
#define MCP_TXB0D0       0x36
#define MCP_RXB0SIDH     0x61
#define MCP_RXB0SIDL     0x62
#define MCP_RXB0DLC      0x65
#define MCP_RXB0D0       0x66
#define MCP_RXB1CTRL     0x70
#define MCP_RXB1SIDH     0x71
#define MCP_RXB1SIDL     0x72
#define MCP_RXB1DLC      0x75
#define MCP_RXB1D0       0x76

/* CANCTRL mode bits */
#define MCP_MODE_NORMAL    0x00
#define MCP_MODE_LOOPBACK  0x40
#define MCP_MODE_CONFIG    0x80
/* CANCTRL One-Shot Mode: no automatic retransmission. Keeps mcp2515_send()
 * from blocking forever when the bus has no other node to ACK (motor off) —
 * a lost torque frame is replaced 1 ms later anyway. */
#define MCP_OSM            0x08

/* Initialize SPI + MCP2515, set baud rate, enter NORMAL mode.
 * Returns true if self-test (read CANSTAT) succeeds. */
bool mcp2515_init(void);

/* Send a standard 11-bit CAN frame. len 0..8. Blocks until TX buffer free. */
bool mcp2515_send(uint16_t id, const uint8_t *data, uint8_t len);

/* Poll for a received standard frame. Returns true if a frame was read. */
bool mcp2515_receive(uint16_t *id, uint8_t *data, uint8_t *len);

/* Quick register read for diagnostics */
uint8_t mcp2515_read_reg(uint8_t addr);

#endif /* MCP2515_H */
