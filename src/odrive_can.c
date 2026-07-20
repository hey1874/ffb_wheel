/*
 * odrive_can.c - ODrive-compatible CAN protocol layer (GIM6010-8)
 *
 * Talks to the SteadyWin GIM6010-8 motor (ODrive-compatible firmware v0.5.16)
 * over CAN via the MCP2515 SPI controller. The motor's PDF manual (p43-44)
 * documents this as standard ODrive CAN.
 *
 * CAN ID (11-bit) = (node_id << 5) | cmd_id
 *
 * Commands implemented:
 *   0x01  Heartbeat            (motor→host)  uint32 error, uint32 current_state
 *   0x07  Set_Axis_State       (host→motor)  uint32 requested_state
 *   0x09  Get_Encoder_Estimates(motor→host)  float pos, float vel
 *   0x0B  Set_Controller_Mode  (host→motor)  uint32 control_mode, uint32 input_mode
 *   0x0E  Set_Input_Torque     (host→motor)  float input_torque (Nm)
 *
 * The motor broadcasts Heartbeat (0x01) and Get_Encoder_Estimates (0x09)
 * periodically (every ~100 ms by default in ODrive firmware). odrive_poll()
 * picks these up and caches the latest values.
 */
#include "odrive_can.h"
#include "mcp2515.h"
#include "pico/time.h"
#include <string.h>

/* If the motor's periodic broadcasts go silent for this long, treat it as
 * offline. The GIM6010-8 defaults are heartbeat 100 ms / encoder 10 ms, so
 * 250 ms tolerates jitter and a couple of dropped frames without false trips. */
#define ODRIVE_TIMEOUT_MS  250u

/* ---- ODrive command IDs ---- */
#define ODRV_CMD_HEARTBEAT          0x01
#define ODRV_CMD_GET_ENCODER_EST    0x09
#define ODRV_CMD_SET_AXIS_STATE     0x07
#define ODRV_CMD_SET_CTRL_MODE      0x0B
#define ODRV_CMD_SET_INPUT_TORQUE   0x0E

/* ---- Axis states ---- */
#define ODRV_AXIS_IDLE              1
#define ODRV_AXIS_CLOSED_LOOP       8

/* ---- Control / input modes ---- */
#define ODRV_CTRL_TORQUE            1
#define ODRV_INPUT_DIRECT           1

/* ---- Cached state (written by odrive_poll on core1, read by the FFB loop on
 * core0). Marked volatile so the compiler always re-reads shared SRAM; on the
 * RP2040 (no data cache) aligned 32-bit fields are also torn-read-free across
 * cores, so plain accessors need no locking. ---- */
static volatile struct {
    float    position;       /* turns, motor side */
    float    velocity;       /* turns/s, motor side */
    uint32_t axis_state;     /* from heartbeat */
    uint32_t axis_error;     /* from heartbeat */
    bool     closed_loop;
    bool     encoder_valid;  /* at least one recent Get_Encoder_Estimates */
    uint32_t last_hb_ms;     /* ms of last heartbeat (0 = never seen) */
    uint32_t last_enc_ms;    /* ms of last encoder estimate (0 = never seen) */
} s_odrv = {
    .position = 0.0f,
    .velocity = 0.0f,
    .axis_state = 0,
    .axis_error = 0,
    .closed_loop = false,
    .encoder_valid = false,
    .last_hb_ms = 0,
    .last_enc_ms = 0,
};

/* ---- Helpers ---- */

static inline uint16_t make_can_id(uint8_t node_id, uint8_t cmd_id) {
    return (uint16_t)((node_id << 5) | (cmd_id & 0x1F));
}

/* Pack a 32-bit value into 4 bytes, little-endian. */
static void pack_u32(uint8_t *dst, uint32_t v) {
    dst[0] = (uint8_t)(v);
    dst[1] = (uint8_t)(v >> 8);
    dst[2] = (uint8_t)(v >> 16);
    dst[3] = (uint8_t)(v >> 24);
}

/* Pack a float into 4 bytes, little-endian (IEEE 754). */
static void pack_f32(uint8_t *dst, float f) {
    /* Type-pun via union to avoid strict-aliasing issues. */
    union { float f; uint32_t u; } pun;
    pun.f = f;
    pack_u32(dst, pun.u);
}

/* Unpack 4 little-endian bytes into a float. */
static float unpack_f32(const uint8_t *src) {
    union { float f; uint32_t u; } pun;
    pun.u = (uint32_t)src[0]
          | ((uint32_t)src[1] << 8)
          | ((uint32_t)src[2] << 16)
          | ((uint32_t)src[3] << 24);
    return pun.f;
}

/* Unpack 4 little-endian bytes into a uint32. */
static uint32_t unpack_u32(const uint8_t *src) {
    return (uint32_t)src[0]
         | ((uint32_t)src[1] << 8)
         | ((uint32_t)src[2] << 16)
         | ((uint32_t)src[3] << 24);
}

/* Send a command with a single uint32 payload. */
static void send_u32_cmd(uint8_t node_id, uint8_t cmd_id, uint32_t value) {
    uint8_t data[8] = {0};
    pack_u32(data, value);
    /* MCP2515 send pads to 8 bytes; ODrive ignores extra bytes. */
    mcp2515_send(make_can_id(node_id, cmd_id), data, 8);
}

/* Send a command with two uint32 payloads (8 bytes total). */
static void send_u32_u32_cmd(uint8_t node_id, uint8_t cmd_id,
                             uint32_t a, uint32_t b) {
    uint8_t data[8];
    pack_u32(data,     a);
    pack_u32(data + 4, b);
    mcp2515_send(make_can_id(node_id, cmd_id), data, 8);
}

/* ---- Public API ---- */

void odrive_request_closed_loop(uint8_t node_id) {
    /* 1. Set controller mode = TORQUE (1), input mode = DIRECT (1). */
    send_u32_u32_cmd(node_id, ODRV_CMD_SET_CTRL_MODE,
                     ODRV_CTRL_TORQUE, ODRV_INPUT_DIRECT);

    /* 2. Put the axis into CLOSED_LOOP (state 8). */
    send_u32_cmd(node_id, ODRV_CMD_SET_AXIS_STATE, ODRV_AXIS_CLOSED_LOOP);

    /* 3. Zero torque so the motor doesn't run away before the FFB engine
     *    takes over. */
    odrive_set_torque(node_id, 0.0f);
}

bool odrive_init(uint8_t node_id) {
    if (!mcp2515_init()) {
        return false;
    }

    /* Fire the first arm request and return immediately. Blocking here for
     * a heartbeat would stall tud_task() right in the USB enumeration
     * window; the main loop re-sends the request until the heartbeat
     * confirms CLOSED_LOOP. */
    odrive_request_closed_loop(node_id);
    return true;
}

void odrive_set_torque(uint8_t node_id, float nm) {
    uint8_t data[8] = {0};
    pack_f32(data, nm);
    mcp2515_send(make_can_id(node_id, ODRV_CMD_SET_INPUT_TORQUE), data, 8);
}

void odrive_poll(void) {
    uint16_t id;
    uint8_t  data[8];
    uint8_t  len;
    uint32_t now = to_ms_since_boot(get_absolute_time());

    /* Drain all pending frames in this poll cycle. */
    while (mcp2515_receive(&id, data, &len)) {
        if (len < 4) {
            continue;  /* too short to be useful */
        }

        uint8_t cmd_id = id & 0x1F;
        /* node_id = (id >> 5) & 0x3F — we accept frames from any node. */

        switch (cmd_id) {
            case ODRV_CMD_HEARTBEAT:
                /* ODrive v0.5.x heartbeat: axis_error (u32), then
                 * current_state as a SINGLE byte, followed by motor/encoder/
                 * controller flag bytes. Reading state as u32 would misfire
                 * whenever any flag byte is nonzero. */
                if (len >= 5) {
                    s_odrv.axis_error = unpack_u32(data);
                    s_odrv.axis_state = data[4];
                    s_odrv.closed_loop = (s_odrv.axis_state == ODRV_AXIS_CLOSED_LOOP);
                    s_odrv.last_hb_ms = now ? now : 1;   /* keep nonzero */
                }
                break;

            case ODRV_CMD_GET_ENCODER_EST:
                /* data[0..3] = pos_estimate (turns), data[4..7] = vel_estimate (turns/s) */
                if (len >= 8) {
                    s_odrv.position = unpack_f32(data);
                    s_odrv.velocity = unpack_f32(data + 4);
                    s_odrv.encoder_valid = true;
                    s_odrv.last_enc_ms = now ? now : 1;
                }
                break;

            default:
                /* Ignore other frames (e.g. Get_Error, Get_Iq, etc.) */
                break;
        }
    }

    /* Offline detection: if the broadcasts stop (motor powered off, cable
     * pulled, bus error) invalidate the cached state. This gates torque off
     * (closed_loop=false), re-fires the arm request, and — because encoder_valid
     * drops — makes the app re-capture wheel center when the motor returns
     * (its encoder zero may have shifted across a power cycle). Unsigned delta
     * is wrap-safe. */
    if (s_odrv.last_hb_ms && (uint32_t)(now - s_odrv.last_hb_ms) > ODRIVE_TIMEOUT_MS) {
        s_odrv.closed_loop = false;
        s_odrv.axis_state  = 0;
    }
    if (s_odrv.last_enc_ms && (uint32_t)(now - s_odrv.last_enc_ms) > ODRIVE_TIMEOUT_MS) {
        s_odrv.encoder_valid = false;
    }
}

float odrive_get_position(void) {
    return s_odrv.position;
}

float odrive_get_velocity(void) {
    return s_odrv.velocity;
}

bool odrive_is_closed_loop(void) {
    return s_odrv.closed_loop;
}

uint32_t odrive_axis_error(void) {
    return s_odrv.axis_error;
}

bool odrive_has_encoder(void) {
    return s_odrv.encoder_valid;
}
