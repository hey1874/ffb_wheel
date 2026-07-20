/*
 * main.c - FFB wheel firmware entry point (RP2040 + TinyUSB + MCP2515 CAN)
 *
 * Wiring:
 *   - USB:  RP2040 native USB (Full-Speed). Games see VID 0x1209/PID 0xFFB0.
 *   - CAN:  MCP2515 SPI-CAN module (spi0: GP18/19/16/17) → GIM6010-8 motor.
 *   - Motor: SteadyWin GIM6010-8 (ODrive-compatible firmware v0.5.16,
 *           node_id 1 default, 8:1 gearbox).
 *
 * Dual-core split (so CAN blocking can never stall USB timing):
 *   core0 (this file's main): USB (tud_task) + FFB effect engine + input report
 *          (wheel axis, pedals ADC, buttons I2C). Publishes torque to a shared
 *          g_torque_cmd; reads the s_odrv encoder/state cache.
 *   core1 (core1_entry): all MCP2515/SPI/CAN — init, poll (fills s_odrv),
 *          1 Hz re-arm, and the ~1 kHz torque stream (reads g_torque_cmd).
 *
 * Data flow:
 *   Game ←USB HID FFB→ TinyUSB → ffb.c → ffb_output_torque() → g_torque_cmd
 *     ─(core1)→ odrive_set_torque() → MCP2515 → CAN → GIM6010-8
 *   GIM6010-8 → CAN → MCP2515 → odrive_poll() (core1) → s_odrv cache
 *     ─(core0)→ read_encoder_*() → ffb_calculate() + wheel input report
 */
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/timer.h"

#include "ffb.h"
#include "odrive_can.h"
#include "pedals.h"
#include "buttons.h"

/* ============================================================ */
/* Configuration                                                */
/* ============================================================ */

/* Output-side (wheel-rim) torque in Nm at the FFB engine's full-scale output
 * (±32767). Override with -DMAX_NM=... at build time.
 *
 * The GIM6010-8's ODrive torque_constant is configured output-referenced, so
 * Set_Input_Torque (0x0E) is in rim Nm with the 8:1 reduction already folded in
 * — MAX_NM is sent as-is (no gearbox division here). 4.0 ≈ the actuator's rated
 * output torque.
 *
 * SAFETY: if a given motor's torque_constant were instead motor-referenced,
 * MAX_NM would be 8× stronger at the rim (4.0 → ~32 Nm, wrist-breaking). Verify
 * once on first bring-up — command a small torque and check the felt force /
 * motor current — before trusting full scale. Start low: -DMAX_NM=0.5. */
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

/* Fail-safe: if core0 (the FFB loop) stops updating its liveness stamp for this
 * long, core1 treats it as wedged and commands zero torque. Without this the
 * dual-core split would keep streaming the last torque AND keep feeding the
 * ODrive watchdog even with a dead FFB loop, leaving the wheel holding force. */
#define CORE0_STALL_MS  50

/* ---- Polarity (bring-up calibration; each is +1 or -1) ---------------------
 * The motor/encoder wiring fixes two physical facts the firmware can't know:
 * which way the encoder counts, and which way positive torque pushes. Correct
 * them here without touching logic. Procedure (do at low MAX_NM, e.g. 0.5):
 *   1. ENCODER_SIGN: turn the wheel right; if the in-game axis moves the wrong
 *      way, flip it. This sets both the game axis AND the spring reference.
 *   2. TORQUE_SIGN:  enable a centering spring; if the wheel is pushed AWAY
 *      from center / runs away instead of returning, flip it.
 * Get ENCODER_SIGN right first (step 1), then TORQUE_SIGN (step 2). */
#ifndef ENCODER_SIGN
#define ENCODER_SIGN  (+1)
#endif
#ifndef TORQUE_SIGN
#define TORQUE_SIGN   (-1)  /* CyberBeast BL72: positive torque = CW rotation */
#endif

/* Capture the wheel's power-on position as center once the encoder goes live
 * (default). Center the wheel before/at power-on, or call wheel_recenter()
 * (e.g. from a button) when it is physically centered. Set 0 to fall back to
 * the ODrive encoder's own zero as center. */
#ifndef WHEEL_CENTER_AT_BOOT
#define WHEEL_CENTER_AT_BOOT  1
#endif

/* Soft zero: raw motor turns treated as wheel center. Subtracted from every
 * position read so "center" is a definable reference, not the arbitrary
 * ODrive/encoder zero (which for an absolute encoder need not be the wheel's
 * mechanical center, and for an incremental one is just the power-on spot). */
static float s_center_offset = 0.0f;
/* Tracks the previous "encoder live" state so center is (re)captured on each
 * rising edge — first boot and any motor power-cycle recovery. */
static bool  s_enc_was_live  = false;

/* Redefine the current wheel position as center. External linkage so a future
 * button handler can rebind it; also called on each encoder-live rising edge. */
void wheel_recenter(void) {
    s_center_offset = odrive_get_position();
}

/* Sign-corrected, center-referenced position: apply the center offset and
 * ENCODER_SIGN at one source so the game axis, spring, damper and inertia all
 * share one consistent frame. Velocity is a rate, unaffected by the offset. */
static inline float enc_position(void) {
    return ENCODER_SIGN * (odrive_get_position() - s_center_offset);
}
static inline float enc_velocity(void) { return ENCODER_SIGN * odrive_get_velocity(); }

/* ============================================================ */
/* Platform override: FFB tick clock                            */
/* ============================================================ */

uint32_t ffb_get_tick_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

/* ============================================================ */
/* Platform override: torque output → shared command for core1  */
/* ============================================================ */

/* Latest engine torque (-32767..32767), produced by the FFB loop on core0 and
 * consumed by the CAN loop on core1. A single aligned 16-bit store is atomic
 * across cores on the RP2040, so no lock is needed; volatile forces the store
 * to memory where core1 will see it. */
static volatile int16_t g_torque_cmd = 0;

/* Core0 liveness stamp (ms of the last main-loop pass). core1 watches it and
 * stops commanding torque if it stops advancing — see CORE0_STALL_MS. */
static volatile uint32_t g_core0_alive_ms = 0;

void ffb_output_torque(int16_t torque) {
    /* Runs on core0. Just publish the command; core1 owns the CAN link and does
     * the Nm/TORQUE_SIGN conversion next to the actual send, so nothing on the
     * USB core ever touches SPI. */
    g_torque_cmd = torque;
}

/* ============================================================ */
/* Encoder reads → ODrive cached position/velocity             */
/* ============================================================ */

/* Track previous velocity for a crude, EMA-smoothed acceleration estimate. */
static float    s_prev_vel      = 0.0f;
static uint32_t s_prev_vel_tick = 0;
static int32_t  s_acc_filt      = 0;   /* lowpassed acceleration, ±10000 */

static int32_t read_encoder_position(void) {
    float pos = enc_position();  /* motor turns, sign-corrected */
    /* Normalize to -10000..10000 (PID condition metric). */
    int32_t v = (int32_t)(pos * (10000.0f / MOTOR_MAX_TURNS));
    if (v >  10000) v =  10000;
    if (v < -10000) v = -10000;
    return v;
}

static int32_t read_encoder_velocity(void) {
    float vel = enc_velocity();  /* motor turns/s, sign-corrected */
    /* Normalize to -10000..10000. */
    int32_t v = (int32_t)(vel * (10000.0f / VEL_NORM_TURNS));
    if (v >  10000) v =  10000;
    if (v < -10000) v = -10000;
    return v;
}

static int32_t read_encoder_acceleration(void) {
    /* Estimate acceleration by differentiating the cached velocity. Two facts
     * make a naive per-call diff useless: the engine calls this at ~1 kHz, but
     * the velocity only refreshes when a CAN encoder frame arrives (~100 Hz on
     * the GIM6010-8 default). So most calls see dv=0 and the ~10th sees the
     * whole step over a 1 ms dt — a ~10x spike. We compute the raw diff every
     * call (dt is >=1 ms here, so no divide-by-zero and no exactly-1 ms drop)
     * and run it through an EMA: the spikes average out to the true rate, and a
     * run of zero-diff samples decays the estimate back to zero on its own. */
    float    vel = enc_velocity();  /* sign-corrected */
    uint32_t now = ffb_get_tick_ms();
    if (s_prev_vel_tick == 0) {
        /* First call: seed the reference, no diff yet. */
        s_prev_vel      = vel;
        s_prev_vel_tick = now ? now : 1;   /* keep nonzero so we stay seeded */
        return 0;
    }
    if (now != s_prev_vel_tick) {
        float   dt = (float)(now - s_prev_vel_tick) / 1000.0f;
        float   dv = (vel - s_prev_vel) / dt;              /* turns/s² */
        int32_t a  = (int32_t)(dv * (10000.0f / 800.0f));  /* 800 turns/s² → 10000 */
        if (a >  10000) a =  10000;
        if (a < -10000) a = -10000;
        /* lowpass, τ≈8 samples; round the /8 so small residuals aren't
         * truncated to a steady-state dead band. */
        int32_t d = a - s_acc_filt;
        s_acc_filt += (d + (d >= 0 ? 4 : -4)) / 8;
        s_prev_vel      = vel;
        s_prev_vel_tick = now;
    }
    return s_acc_filt;
}

/* Wheel axis position for the input report (int16_t, -32767..32767). */
static int16_t read_wheel_axis(void) {
    float pos = enc_position();  /* motor turns, sign-corrected */
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
/* Core 1: CAN link (MCP2515 / ODrive)                          */
/*                                                              */
/* All MCP2515 / SPI / CAN traffic lives here so its blocking    */
/* (init sleeps, per-frame SPI, arbitration) can never stall the */
/* USB tud_task() loop on core0. Core0 ↔ core1 share only a few  */
/* single-word values (g_torque_cmd out, s_odrv cache in).       */
/* ============================================================ */

static void core1_entry(void) {
    /* MCP2515 init blocks for tens of ms (oscillator + mode changes) — fine
     * here, it no longer sits in the USB enumeration window on core0. */
    odrive_init(ODRIVE_NODE_ID);

    uint32_t last_arm = 0;
    uint32_t last_tx  = 0;

    while (1) {
        /* Poll incoming CAN frames (encoder estimates + heartbeat). */
        odrive_poll();

        uint32_t now = to_ms_since_boot(get_absolute_time());

        /* Re-arm at 1 Hz until the heartbeat confirms CLOSED_LOOP, so a
         * motor powered up (or power-cycled) after the Pico still arms. */
        if (!odrive_is_closed_loop() && now - last_arm >= 1000) {
            last_arm = now;
            odrive_request_closed_loop(ODRIVE_NODE_ID);
        }

        /* Stream the latest torque at ~1 kHz (also feeds the ODrive CAN
         * watchdog). Map -32767..32767 → ±MAX_NM (rim Nm); TORQUE_SIGN corrects
         * physical rotation direction (see polarity notes above).
         * Fail-safe: if core0's FFB loop wedged (stamp stopped advancing), send
         * zero instead of latching its last torque forever. */
        if (now != last_tx) {
            last_tx = now;
            int16_t t = ((uint32_t)(now - g_core0_alive_ms) > CORE0_STALL_MS)
                        ? 0 : g_torque_cmd;
            float nm = (float)t * (MAX_NM / 32767.0f) * TORQUE_SIGN;
            odrive_set_torque(ODRIVE_NODE_ID, nm);
        }
    }
}

/* ============================================================ */
/* Core 0: USB + FFB engine                                     */
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
    buttons_init();

    /* Hand the CAN link to core1 (MCP2515 init + torque stream + polling). */
    multicore_launch_core1(core1_entry);

    uint32_t last_calc   = 0;
    uint32_t last_report = 0;
    uint32_t last_led    = 0;
    bool     led_state   = false;

    while (1) {
        tud_task();  /* TinyUSB device task — must be called frequently */

        uint32_t now = ffb_get_tick_ms();
        g_core0_alive_ms = now;   /* liveness stamp for core1's fail-safe */

        /* (Re)capture center on every rising edge of "encoder live". This fires
         * at first boot AND whenever the motor returns after a power-cycle —
         * its encoder zero may have shifted, so re-centering here (before torque
         * is re-enabled below) stops a stale offset from slamming the wheel.
         * odrive_has_encoder() reads the core1-updated cache and drops to false
         * on broadcast timeout. */
        bool enc_live = odrive_has_encoder();
        if (WHEEL_CENTER_AT_BOOT && enc_live && !s_enc_was_live) {
            wheel_recenter();
        }
        s_enc_was_live = enc_live;

        /* Effect engine at ~1 kHz. Reads encoder cache, sums effects, and
         * publishes torque via ffb_output_torque() → g_torque_cmd (core1 sends). */
        if (now != last_calc) {
            last_calc = now;
            /* Drive torque only when the motor is armed and healthy AND the
             * encoder is live. Requiring odrive_has_encoder() closes a narrow
             * window after a power-cycle where a CLOSED_LOOP heartbeat could
             * arrive before the first encoder frame re-validates (and thus
             * re-centers) — otherwise torque could push against a stale center.
             * A fault / not-CLOSED_LOOP / offline timeout all command zero. */
            if (odrive_axis_error() != 0 || !odrive_is_closed_loop()
                || !odrive_has_encoder()) {
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
                 * compile-time constants, so this is safe for any PEDAL_COUNT.
                 * Size ≥1 so PEDAL_COUNT==0 isn't a zero-length array. */
                int16_t ped[PEDAL_COUNT > 0 ? PEDAL_COUNT : 1] = {0};
                pedals_read(ped);
                int16_t thr = (PEDAL_COUNT > 0) ? ped[0] : 0;
                int16_t brk = (PEDAL_COUNT > 1) ? ped[1] : 0;
                int16_t clu = (PEDAL_COUNT > 2) ? ped[2] : 0;
                uint8_t buf[sizeof(ffb_wheel_report_t)];
                ffb_build_wheel_report(buf, sizeof(buf), buttons_read(),
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
