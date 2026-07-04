/*
 * ffb.c - FFB report handler + effect engine (RP2040 / TinyUSB port)
 *
 * Ported to C from VNWheel (MIT, Hoan Tran) and OpenFFBoard (GPL, Yannick).
 * Report dispatch follows VNWheel's FfbOnUsbData; effect math follows
 * OpenFFBoard's EffectsCalculator (16-bit torque range, PID-correct
 * sawtooth up/down ordering).
 */
#include "ffb.h"
#include <string.h>
#include <math.h>

/* ---- Platform time hook (override in main.c with pico SDK timer) ---- */
uint32_t ffb_get_tick_ms(void) __attribute__((weak));
uint32_t ffb_get_tick_ms(void) { return 0; }

/* ---- Weak torque output (override in motor driver code) ---- */
void ffb_output_torque(int16_t torque) __attribute__((weak));
void ffb_output_torque(int16_t torque) { (void)torque; }

/* ---- Global state ---- */
static ffb_effect_t       g_effects[FFB_MAX_EFFECTS + 1]; /* 1-based indexing */
static uint8_t            g_next_eid = 1;
static bool               g_device_paused = false;
static bool               g_actuators_enabled = true;
static uint8_t            g_device_gain = 255;

static ffb_pid_state_report_t  g_pid_state;
static ffb_block_load_t        g_block_load;
static ffb_pid_pool_t          g_pid_pool;

/* ============================================================ */
/* Effect management                                            */
/* ============================================================ */

static void stop_effect(uint8_t id) {
    if (id > FFB_MAX_EFFECTS) return;
    g_effects[id].state &= ~FFB_STATE_PLAYING;
}

static void stop_all_effects(void) {
    for (uint8_t id = 0; id <= FFB_MAX_EFFECTS; id++)
        stop_effect(id);
}

static void free_effect(uint8_t id) {
    if (id > FFB_MAX_EFFECTS) return;
    memset(&g_effects[id], 0, sizeof(ffb_effect_t));
    if (id < g_next_eid) g_next_eid = id;
}

static void free_all_effects(void) {
    g_next_eid = 1;
    memset(g_effects, 0, sizeof(g_effects));
    g_block_load.ramPoolAvailable = sizeof(g_effects);
}

static uint8_t get_next_free_effect(void) {
    /* Scan forward from g_next_eid to find a free slot. */
    while (g_next_eid <= FFB_MAX_EFFECTS &&
           g_effects[g_next_eid].state != FFB_STATE_FREE) {
        g_next_eid++;
    }
    if (g_next_eid > FFB_MAX_EFFECTS) return 0;
    uint8_t id = g_next_eid;
    g_effects[id].state = FFB_STATE_ALLOCATED;
    g_next_eid++;  /* advance past this slot for the next search */
    return id;
}

static void start_effect(uint8_t id) {
    if (id > FFB_MAX_EFFECTS) return;
    g_effects[id].state = FFB_STATE_PLAYING | FFB_STATE_ALLOCATED;
    g_effects[id].startTime = ffb_get_tick_ms();
    g_effects[id].elapsedTime = 0;
}

/* ============================================================ */
/* SET_REPORT handlers                                          */
/* ============================================================ */

static void handle_set_effect(const ffb_set_effect_t *r) {
    if (r->effectBlockIndex > FFB_MAX_EFFECTS) return;
    ffb_effect_t *e = &g_effects[r->effectBlockIndex];
    e->effectType  = r->effectType;
    e->duration    = r->duration;
    e->gain        = r->gain ? r->gain : 255;
    /* directionX/Y, enableAxis, samplePeriod, triggerRepeatInterval stored
     * implicitly; single-axis wheel applies everything on X. */
}

static void handle_set_envelope(const ffb_set_envelope_t *r) {
    if (r->effectBlockIndex > FFB_MAX_EFFECTS) return;
    ffb_effect_t *e = &g_effects[r->effectBlockIndex];
    e->attackLevel  = (int16_t)r->attackLevel;
    e->fadeLevel    = (int16_t)r->fadeLevel;
    e->attackTime   = r->attackTime;
    e->fadeTime     = r->fadeTime;
}

static void handle_set_condition(const ffb_set_condition_t *r) {
    if (r->effectBlockIndex > FFB_MAX_EFFECTS) return;
    ffb_effect_t *e = &g_effects[r->effectBlockIndex];
    e->cpOffset              = r->cpOffset;
    e->positiveCoefficient  = r->positiveCoefficient;
    e->negativeCoefficient  = r->negativeCoefficient;
    e->positiveSaturation   = r->positiveSaturation;
    e->negativeSaturation   = r->negativeSaturation;
    e->deadBand             = r->deadBand;
}

static void handle_set_periodic(const ffb_set_periodic_t *r) {
    if (r->effectBlockIndex > FFB_MAX_EFFECTS) return;
    ffb_effect_t *e = &g_effects[r->effectBlockIndex];
    e->magnitude = (int16_t)r->magnitude;
    e->offset    = r->offset;
    e->phase     = r->phase;
    e->period    = r->period;
}

static void handle_set_constant(const ffb_set_constant_t *r) {
    if (r->effectBlockIndex > FFB_MAX_EFFECTS) return;
    g_effects[r->effectBlockIndex].magnitude = r->magnitude;
}

static void handle_set_ramp(const ffb_set_ramp_t *r) {
    if (r->effectBlockIndex > FFB_MAX_EFFECTS) return;
    g_effects[r->effectBlockIndex].startMagnitude = r->startMagnitude;
    g_effects[r->effectBlockIndex].endMagnitude   = r->endMagnitude;
}

static void handle_effect_op(const ffb_effect_op_t *r) {
    if (r->effectBlockIndex > FFB_MAX_EFFECTS) return;
    ffb_effect_t *e = &g_effects[r->effectBlockIndex];
    switch (r->operation) {
        case 1: /* Start */
            if (r->loopCount > 1 && e->duration != FFB_DURATION_INFINITE)
                e->duration *= r->loopCount;
            if (r->loopCount == 0xFF) e->duration = FFB_DURATION_INFINITE;
            start_effect(r->effectBlockIndex);
            break;
        case 2: /* StartSolo */
            stop_all_effects();
            start_effect(r->effectBlockIndex);
            break;
        case 3: /* Stop */
            stop_effect(r->effectBlockIndex);
            break;
        default: break;
    }
}

static void handle_block_free(const ffb_block_free_t *r) {
    if (r->effectBlockIndex == 0xFF) free_all_effects();
    else                              free_effect(r->effectBlockIndex);
}

static void handle_device_control(const ffb_device_control_t *r) {
    switch (r->control) {
        case FFB_DC_ENABLE_ACTUATORS:  g_actuators_enabled = true;  break;
        case FFB_DC_DISABLE_ACTUATORS: g_actuators_enabled = false; break;
        case FFB_DC_STOP:              stop_all_effects();          break;
        case FFB_DC_RESET:             free_all_effects();          break;
        case FFB_DC_PAUSE:             g_device_paused = true;      break;
        case FFB_DC_CONTINUE:          g_device_paused = false;     break;
        default: break;
    }
}

static void handle_device_gain(const ffb_device_gain_t *r) {
    g_device_gain = r->gain;
}

/* Feature SET: create new effect */
static void handle_create_effect(const ffb_create_effect_t *r) {
    (void)r; /* effectType tracked via subsequent Set Effect report */
    g_block_load.effectBlockIndex = get_next_free_effect();
    g_block_load.loadStatus = g_block_load.effectBlockIndex ? 1 : 2;
    if (g_block_load.effectBlockIndex) {
        memset(&g_effects[g_block_load.effectBlockIndex], 0, sizeof(ffb_effect_t));
        g_effects[g_block_load.effectBlockIndex].state = FFB_STATE_ALLOCATED;
    }
}

/* ============================================================ */
/* SET_REPORT dispatch                                          */
/* ============================================================ */

void ffb_on_set_report(uint8_t report_id, uint8_t report_type,
                       const uint8_t *buffer, uint16_t bufsize) {
    (void)bufsize;
    if (report_type == FFB_REPORT_TYPE_FEATURE) {
        if (report_id == FFB_RID_CREATE_EFFECT)
            handle_create_effect((const ffb_create_effect_t *)buffer);
        return;
    }
    /* FFB_REPORT_TYPE_OUTPUT — fall through to switch below */
    switch (report_id) {
        case FFB_RID_SET_EFFECT:    handle_set_effect((const ffb_set_effect_t *)buffer); break;
        case FFB_RID_SET_ENVELOPE:  handle_set_envelope((const ffb_set_envelope_t *)buffer); break;
        case FFB_RID_SET_CONDITION: handle_set_condition((const ffb_set_condition_t *)buffer); break;
        case FFB_RID_SET_PERIODIC:  handle_set_periodic((const ffb_set_periodic_t *)buffer); break;
        case FFB_RID_SET_CONSTANT:  handle_set_constant((const ffb_set_constant_t *)buffer); break;
        case FFB_RID_SET_RAMP:      handle_set_ramp((const ffb_set_ramp_t *)buffer); break;
        case FFB_RID_EFFECT_OP:     handle_effect_op((const ffb_effect_op_t *)buffer); break;
        case FFB_RID_BLOCK_FREE:    handle_block_free((const ffb_block_free_t *)buffer); break;
        case FFB_RID_DEVICE_CTRL:   handle_device_control((const ffb_device_control_t *)buffer); break;
        case FFB_RID_DEVICE_GAIN:   handle_device_gain((const ffb_device_gain_t *)buffer); break;
        /* custom force data / download sample / set custom — stubbed */
        default: break;
    }
}

/* ============================================================ */
/* GET_REPORT handlers (Feature)                                */
/* ============================================================ */

uint16_t ffb_on_get_report(uint8_t report_id, uint8_t report_type,
                           uint8_t *buffer, uint16_t reqlen) {
    (void)report_type;
    switch (report_id) {
        case FFB_RID_BLOCK_LOAD: {
            if (reqlen < sizeof(ffb_block_load_t)) return 0;
            memcpy(buffer, &g_block_load, sizeof(ffb_block_load_t));
            return sizeof(ffb_block_load_t);
        }
        case FFB_RID_PID_POOL: {
            if (reqlen < sizeof(ffb_pid_pool_t)) return 0;
            g_pid_pool.ramPoolSize = sizeof(g_effects);
            g_pid_pool.maxSimultaneousEffects = FFB_MAX_EFFECTS;
            g_pid_pool.memoryManagement = 0x03; /* device-managed + shared */
            memcpy(buffer, &g_pid_pool, sizeof(ffb_pid_pool_t));
            return sizeof(ffb_pid_pool_t);
        }
        default:
            return 0;
    }
}

/* ============================================================ */
/* Effect math                                                  */
/* ============================================================ */

static int32_t clip32(int32_t v, int32_t lo, int32_t hi) {
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* Envelope scaler (0..32767) applied to periodic forces. */
static int32_t apply_envelope(const ffb_effect_t *e, int32_t force) {
    int32_t mag   = (int32_t)e->magnitude * e->gain / 255;
    int32_t atkL  = (int32_t)e->attackLevel * e->gain / 255;
    int32_t fadeL = (int32_t)e->fadeLevel   * e->gain / 255;
    int32_t scaler = mag;
    uint32_t el = e->elapsedTime;
    if (e->attackTime && el < e->attackTime) {
        scaler = atkL + (mag - atkL) * (int32_t)el / (int32_t)e->attackTime;
    } else if (e->fadeTime && e->duration != FFB_DURATION_INFINITE &&
               el > (uint32_t)(e->duration - e->fadeTime)) {
        uint32_t remaining = e->duration - el;
        scaler = fadeL + (mag - fadeL) * (int32_t)remaining / (int32_t)e->fadeTime;
    }
    return force * scaler / 32767;
}

static int32_t calc_constant(const ffb_effect_t *e) {
    return (int32_t)e->magnitude * e->gain / 255;
}

static int32_t calc_ramp(const ffb_effect_t *e) {
    if (e->duration == 0) return e->startMagnitude;
    return e->startMagnitude +
           (int32_t)e->elapsedTime * (e->endMagnitude - e->startMagnitude) / (int32_t)e->duration;
}

static int32_t calc_square(const ffb_effect_t *e) {
    if (e->period == 0) return 0;
    uint32_t phase_t = (uint32_t)e->phase * e->period / 255;
    uint32_t t = (e->elapsedTime + phase_t) % e->period;
    int32_t hi = (int32_t)e->offset * 2 + e->magnitude;
    int32_t lo = (int32_t)e->offset * 2 - e->magnitude;
    int32_t f = (t < e->period / 2) ? hi : lo;
    return apply_envelope(e, f);
}

static int32_t calc_sine(const ffb_effect_t *e) {
    if (e->period == 0) return 0;
    float angle = (2.0f * 3.14159265f * (float)e->elapsedTime) / (float)e->period
                + (float)e->phase * 2.0f * 3.14159265f / 255.0f;
    float f = (int16_t)e->offset * 2 + sinf(angle) * e->magnitude;
    return apply_envelope(e, (int32_t)f);
}

static int32_t calc_triangle(const ffb_effect_t *e) {
    if (e->period == 0) return 0;
    uint32_t phase_t = (uint32_t)e->phase * e->period / 255;
    uint32_t rem = (e->elapsedTime + phase_t) % e->period;
    int32_t hi = (int16_t)e->offset * 2 + e->magnitude;
    int32_t lo = (int16_t)e->offset * 2 - e->magnitude;
    float slope = (float)(hi - lo) * 2.0f / (float)e->period;
    float f = (rem < e->period / 2) ? (lo + slope * rem)
                                    : (hi + slope * ((float)e->period - rem));
    return apply_envelope(e, (int32_t)f);
}

static int32_t calc_sawtooth_up(const ffb_effect_t *e) {
    if (e->period == 0) return 0;
    uint32_t phase_t = (uint32_t)e->phase * e->period / 255;
    uint32_t rem = (e->elapsedTime + phase_t) % e->period;
    int32_t hi = (int16_t)e->offset * 2 + e->magnitude;
    int32_t lo = (int16_t)e->offset * 2 - e->magnitude;
    float slope = (float)(hi - lo) / (float)e->period;
    int32_t f = (int32_t)(lo + slope * rem);
    return apply_envelope(e, f);
}

static int32_t calc_sawtooth_down(const ffb_effect_t *e) {
    if (e->period == 0) return 0;
    uint32_t phase_t = (uint32_t)e->phase * e->period / 255;
    uint32_t rem = (e->elapsedTime + phase_t) % e->period;
    int32_t hi = (int16_t)e->offset * 2 + e->magnitude;
    int32_t lo = (int16_t)e->offset * 2 - e->magnitude;
    float slope = (float)(hi - lo) / (float)e->period;
    int32_t f = (int32_t)(hi - slope * rem);
    return apply_envelope(e, f);
}

/* Condition force: spring/damper/inertia/friction share this.
 * metric is the relevant axis signal normalized to -10000..10000
 * (matching the PID descriptor's condition parameter range). */
static int32_t calc_condition(const ffb_effect_t *e, int32_t metric) {
    int32_t cp  = e->cpOffset;
    int32_t db  = e->deadBand;
    int32_t f   = 0;
    if (metric < cp - db) {
        f = (metric - (cp - db)) * (-(int32_t)e->negativeCoefficient) / 10000;
    } else if (metric > cp + db) {
        f = (metric - (cp + db)) * (int32_t)e->positiveCoefficient / 10000;
    }
    f = clip32(f, -(int32_t)e->negativeSaturation, (int32_t)e->positiveSaturation);
    return f * e->gain / 255;
}

/* ============================================================ */
/* Engine tick                                                  */
/* ============================================================ */

void ffb_calculate(const ffb_axis_metrics_t *m) {
    int32_t force = 0;
    uint32_t now = ffb_get_tick_ms();

    if (g_device_paused || !g_actuators_enabled) {
        ffb_output_torque(0);
        return;
    }

    for (uint8_t id = 1; id <= FFB_MAX_EFFECTS; id++) {
        ffb_effect_t *e = &g_effects[id];
        if (!(e->state & FFB_STATE_PLAYING)) continue;
        e->elapsedTime = now - e->startTime;
        if (e->duration != FFB_DURATION_INFINITE && e->elapsedTime > e->duration) {
            stop_effect(id);
            continue;
        }
        switch (e->effectType) {
            case FFB_EFFECT_CONSTANT:      force += calc_constant(e); break;
            case FFB_EFFECT_RAMP:          force += calc_ramp(e); break;
            case FFB_EFFECT_SQUARE:        force += calc_square(e); break;
            case FFB_EFFECT_SINE:          force += calc_sine(e); break;
            case FFB_EFFECT_TRIANGLE:      force += calc_triangle(e); break;
            case FFB_EFFECT_SAWTOOTHUP:    force += calc_sawtooth_up(e); break;
            case FFB_EFFECT_SAWTOOTHDOWN:  force += calc_sawtooth_down(e); break;
            case FFB_EFFECT_SPRING:        force += calc_condition(e, m->position); break;
            case FFB_EFFECT_DAMPER:        force += calc_condition(e, m->velocity); break;
            case FFB_EFFECT_INERTIA:       force += calc_condition(e, m->acceleration); break;
            case FFB_EFFECT_FRICTION:      force += calc_condition(e, m->velocity); break;
            default: break;
        }
    }

    force = (int32_t)(force * (int32_t)g_device_gain / 255);
    force = clip32(force, FFB_TORQUE_MIN, FFB_TORQUE_MAX);
    ffb_output_torque((int16_t)force);
}

/* ============================================================ */
/* Input report                                                 */
/* ============================================================ */

uint16_t ffb_build_wheel_report(uint8_t *buffer, uint16_t reqlen,
                                uint8_t buttons,
                                int16_t x, int16_t y, int16_t z,
                                int16_t rx, int16_t ry, int16_t rz) {
    if (reqlen < sizeof(ffb_wheel_report_t)) return 0;
    ffb_wheel_report_t r = {
        .buttons = buttons,
        .xAxis = x, .yAxis = y, .zAxis = z,
        .rxAxis = rx, .ryAxis = ry, .rzAxis = rz
    };
    memcpy(buffer, &r, sizeof(r));
    return sizeof(r);
}

/* ============================================================ */
/* Init                                                         */
/* ============================================================ */

void ffb_init(void) {
    g_next_eid = 1;
    g_device_paused = false;
    g_actuators_enabled = true;
    g_device_gain = 255;
    memset(g_effects, 0, sizeof(g_effects));
    g_block_load.effectBlockIndex = 0;
    g_block_load.loadStatus = 1;
    g_block_load.ramPoolAvailable = sizeof(g_effects);
    g_pid_state.status = FFB_STATUS_ACTUATORS | FFB_STATUS_POWER;
    g_pid_state.effectBlockIndex = 0;
}
