/*
 * ffb.h - Force Feedback engine interface
 *
 * Two integration points for the application:
 *   1. ffb_axis_metrics_t  — fill each loop with current wheel position/velocity
 *      (normalized to -10000..10000). The engine reads it for condition effects.
 *   2. ffb_output_torque() — weak hook called every calc tick with the final
 *      torque (-32767..32767). Override it to drive your CAN motor controller.
 */
#ifndef FFB_H
#define FFB_H

#include <stdint.h>
#include <stdbool.h>
#include "ffb_types.h"

#define FFB_TORQUE_MAX  32767
#define FFB_TORQUE_MIN  (-32767)

/* Axis metrics, normalized to -10000..10000. Updated by the app each tick. */
typedef struct {
    int32_t position;
    int32_t velocity;
    int32_t acceleration;
} ffb_axis_metrics_t;

/* ---- Lifecycle ---- */
void ffb_init(void);

/* ---- USB callback entry points (call from tud_hid_set/get_report_cb) ---- */
/* buffer has the reportId byte already stripped by TinyUSB. */
void     ffb_on_set_report(uint8_t report_id, uint8_t report_type,
                           const uint8_t *buffer, uint16_t bufsize);
uint16_t ffb_on_get_report(uint8_t report_id, uint8_t report_type,
                           uint8_t *buffer, uint16_t reqlen);

/* ---- Effect engine ---- */
/* Run one calculation tick. Reads metrics, sums active effects, clips, and
 * calls ffb_output_torque(). Call at ~1 kHz. */
void ffb_calculate(const ffb_axis_metrics_t *metrics);

/* Build the periodic wheel input report (axes + buttons). Returns bytes
 * written (0 if buffer too small). */
uint16_t ffb_build_wheel_report(uint8_t *buffer, uint16_t reqlen,
                                uint8_t buttons,
                                int16_t x, int16_t y, int16_t z,
                                int16_t rx, int16_t ry, int16_t rz);

/* ---- Weak hook: override in your motor driver code ---- */
void ffb_output_torque(int16_t torque);

#endif /* FFB_H */
