/*
 * main.c - FFB wheel firmware entry point (RP2040 + TinyUSB + MCP2515 CAN)
 *
 * Wiring:
 *   - USB:  RP2040 native USB (Full-Speed). Games see VID 0x1209/PID 0xFFB0.
 *   - CAN:  MCP2515 SPI-CAN module (spi0: GP18/19/16/17) → GIM6010-8 motor.
 *   - Motor: SteadyWin GIM6010-8 (ODrive-compatible firmware v0.5.16,
 *           node_id 0, 8:1 gearbox, 5 Nm motor-side rated torque).
 *
 * Data flow:
 *   Game ←USB HID FFB→ TinyUSB → ffb.c (effect engine) → ffb_output_torque()
 *     → odrive_set_torque() → MCP2515 → CAN → GIM6010-8
 *   GIM6010-8 → CAN → MCP2515 → odrive_poll() → cached pos/vel
 *     → read_encoder_*() → ffb_calculate() + wheel input report
 */
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/timer.h"

#include "ffb.h"
#include "odrive_can.h"
#include "pedals.h"

/* ============================================================ */
/* Configuration                                                */
/* ============================================================ */

/* Motor-side torque (Nm) corresponding to FFB engine's full-scale output
 * (±32767). Override with -DMAX_NM=... at build time.
 *
 * SAFETY — verify the unit before raising this. Set_Input_Torque (0x0E) is
 * in the units of the motor's configured torque constant, normally BEFORE
 * the 8:1 gearbox: the rim then sees up to 8 × MAX_NM (minus losses). If
 * the "5 Nm rated" figure on the GIM6010-8 datasheet is the OUTPUT-side
 * rating, the motor-side rating is only ~0.6 Nm and MAX_NM=4.0 overdrives
 * it. Check the manual and measure current before trusting full scale. */
#ifndef MAX_NM
#define MAX_NM  4.0f
#endif

/* Steering wheel physical range (output-side turns). ±2 turns = ±720°.
 * Used to normalize motor position to the descriptor's int16 axis range
 * and to the PID -10000..10000 condition metric range. */
#ifndef WHEEL_MAX_TURNS
#define WHEEL_MAX_TURNS  2.0f
#endif

/* Gearbox ratio: motor turns per output turn. */
#define GEAR_RATIO  8.0f

/* Motor-side turns corresponding to full wheel lock. */
#define MOTOR_MAX_TURNS  (WHEEL_MAX_TURNS * GEAR_RATIO)  /* 16.0 */

/* Velocity normalization: motor turns/s at which the PID metric hits ±10000.
 * 80 turns/s motor = 10 turns/s output = 3600°/s — faster than any human. */
#define VEL_NORM_TURNS  80.0f

/* ============================================================ */
/* Platform override: FFB tick clock                            */
/* ============================================================ */

uint32_t ffb_get_tick_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

/* ============================================================ */
/* Platform override: torque output → ODrive CAN               */
/* ============================================================ */

void ffb_output_torque(int16_t torque) {
    /* Map -32767..32767 to -MAX_NM..+MAX_Nm (motor side). */
    float nm = (float)torque * (MAX_NM / 32767.0f);
    odrive_set_torque(ODRIVE_NODE_ID, nm);
}

/* ============================================================ */
/* Encoder reads → ODrive cached position/velocity             */
/* ============================================================ */

/* Track previous velocity for crude acceleration estimate. */
static float s_prev_vel = 0.0f;
static uint32_t s_prev_vel_tick = 0;

static int32_t read_encoder_position(void) {
    float pos = odrive_get_position();  /* motor turns */
    /* Normalize to -10000..10000 (PID condition metric). */
    int32_t v = (int32_t)(pos * (10000.0f / MOTOR_MAX_TURNS));
    if (v >  10000) v =  10000;
    if (v < -10000) v = -10000;
    return v;
}

static int32_t read_encoder_velocity(void) {
    float vel = odrive_get_velocity();  /* motor turns/s */
    /* Normalize to -10000..10000. */
    int32_t v = (int32_t)(vel * (10000.0f / VEL_NORM_TURNS));
    if (v >  10000) v =  10000;
    if (v < -10000) v = -10000;
    return v;
}

static int32_t read_encoder_acceleration(void) {
    /* Estimate acceleration from velocity delta. The FFB engine calls this
     * at ~1 kHz, so dt ≈ 1 ms. We use the cached velocity; the Inertia
     * effect is rarely used and this rough estimate is good enough. */
    float vel = odrive_get_velocity();
    uint32_t now = ffb_get_tick_ms();
    int32_t acc = 0;
    if (s_prev_vel_tick != 0 && now != s_prev_vel_tick) {
        float dt = (float)(now - s_prev_vel_tick) / 1000.0f;
        if (dt > 0.001f) {
            float dv = (vel - s_prev_vel) / dt;  /* turns/s² */
            /* Normalize: 800 turns/s² → 10000. */
            acc = (int32_t)(dv * (10000.0f / 800.0f));
            if (acc >  10000) acc =  10000;
            if (acc < -10000) acc = -10000;
        }
    }
    s_prev_vel = vel;
    s_prev_vel_tick = now;
    return acc;
}

/* Wheel axis position for the input report (int16_t, -32767..32767). */
static int16_t read_wheel_axis(void) {
    float pos = odrive_get_position();  /* motor turns */
    int32_t v = (int32_t)(pos * (32767.0f / MOTOR_MAX_TURNS));
    if (v >  32767) v =  32767;
    if (v < -32767) v = -32767;
    return (int16_t)v;
}

/* ============================================================ */
/* TinyUSB device callbacks — safety: never keep pushing after   */
/* the host goes away (game crash, unplug, suspend).             */
/* ============================================================ */

void tud_umount_cb(void) {
    ffb_stop_all();
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    ffb_stop_all();
}

/* ============================================================ */
/* TinyUSB HID callbacks → FFB engine                           */
/* ============================================================ */

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen) {
    (void)itf;
    return ffb_on_get_report(report_id, report_type, buffer, reqlen);
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize) {
    (void)itf;
    /* OUT-endpoint data arrives with report_id == 0; the first byte is the
     * report ID. Control-transfer SET_REPORT arrives with report_id set and
     * the byte already stripped (see hid_device.c:317-321). Unify both. */
    if (report_id == 0 && bufsize > 0) {
        report_id = buffer[0];
        buffer++;
        bufsize--;
    }
    ffb_on_set_report(report_id, (uint8_t)report_type, buffer, bufsize);
}

/* ============================================================ */
/* Main loop                                                    */
/* ============================================================ */

int main(void) {
    board_init();

    tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO
    };
    tusb_init(0, &dev_init);
    board_init_after_tusb();

    ffb_init();
    pedals_init();

    /* Initialize MCP2515 + ODrive (puts motor in CLOSED_LOOP + TORQUE mode). */
    odrive_init(ODRIVE_NODE_ID);

    uint32_t last_calc   = 0;
    uint32_t last_report = 0;
    uint32_t last_led    = 0;
    uint32_t last_arm    = 0;
    bool     led_state   = false;

    while (1) {
        tud_task();  /* TinyUSB device task — must be called frequently */

        /* Poll incoming CAN frames (encoder estimates + heartbeat). */
        odrive_poll();

        uint32_t now = ffb_get_tick_ms();

        /* Re-arm at 1 Hz until the heartbeat confirms CLOSED_LOOP, so a
         * motor powered up (or power-cycled) after the Pico still arms. */
        if (!odrive_is_closed_loop() && now - last_arm >= 1000) {
            last_arm = now;
            odrive_request_closed_loop(ODRIVE_NODE_ID);
        }

        /* Effect engine at ~1 kHz. Reads encoder, sums effects, calls
         * ffb_output_torque(). */
        if (now != last_calc) {
            last_calc = now;
            if (odrive_axis_error() != 0) {
                /* Motor faulted: command zero torque, don't fight it. */
                ffb_output_torque(0);
            } else {
                ffb_axis_metrics_t m = {
                    .position     = read_encoder_position(),
                    .velocity     = read_encoder_velocity(),
                    .acceleration = read_encoder_acceleration()
                };
                ffb_calculate(&m);
            }
        }

        /* Wheel input report at ~250 Hz (4 ms). Games poll the axis here. */
        if (now - last_report >= 4) {
            last_report = now;
            if (tud_hid_ready()) {
                int16_t x = read_wheel_axis();
                /* Pedals → Y (throttle), Z (brake), Rx (clutch). Guards are
                 * compile-time constants, so this is safe for any PEDAL_COUNT. */
                int16_t ped[PEDAL_COUNT] = {0};
                pedals_read(ped);
                int16_t thr = (PEDAL_COUNT > 0) ? ped[0] : 0;
                int16_t brk = (PEDAL_COUNT > 1) ? ped[1] : 0;
                int16_t clu = (PEDAL_COUNT > 2) ? ped[2] : 0;
                uint8_t buf[sizeof(ffb_wheel_report_t)];
                ffb_build_wheel_report(buf, sizeof(buf), 0,
                                        x, thr, brk, clu, 0, 0);
                tud_hid_report(FFB_RID_WHEEL, buf, sizeof(buf));
            }
        }

        /* LED heartbeat: 1 Hz when mounted, 4 Hz when not. */
        uint32_t led_interval = tud_mounted() ? 500 : 250;
        if (now - last_led >= led_interval) {
            last_led = now;
            board_led_write(led_state);
            led_state = !led_state;
        }
    }
}
