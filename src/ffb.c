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

void ffb_stop_all(void) {
    stop_all_effects();
    ffb_output_torque(0);
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
    e->gain        = r->gain;
    e->enableAxis  = r->enableAxis;
    e->directionX  = r->directionX;
    /* Games may re-send Set Effect while the effect is playing; keep the
     * running total in sync (loopCount is re-applied on the next Start op). */
    e->totalDuration = FFB_DURATION_IS_INF(r->duration)
                     ? FFB_TOTALDUR_INFINITE : r->duration;
    /* samplePeriod / triggerRepeatInterval unused on this device. */
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
    /* Low nibble = Parameter Block Offset. This wheel has one FFB axis (X,
     * block 0); a Y-axis condition block (offset 1) must not clobber it. */
    if ((r->parameterBlockOffset & 0x0F) != 0) return;
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

/* Total playback time for a Start op: duration × loopCount, without
 * mutating the stored per-iteration duration (Start may repeat). */
static uint32_t total_duration_for(const ffb_effect_t *e, uint8_t loopCount) {
    if (FFB_DURATION_IS_INF(e->duration) || loopCount == 0xFF)
        return FFB_TOTALDUR_INFINITE;
    uint8_t loops = loopCount ? loopCount : 1;
    return (uint32_t)e->duration * loops;
}

static void handle_effect_op(const ffb_effect_op_t *r) {
    if (r->effectBlockIndex > FFB_MAX_EFFECTS) return;
    ffb_effect_t *e = &g_effects[r->effectBlockIndex];
    switch (r->operation) {
        case 1: /* Start */
            e->totalDuration = total_duration_for(e, r->loopCount);
            start_effect(r->effectBlockIndex);
            break;
        case 2: /* StartSolo */
            stop_all_effects();
            e->totalDuration = total_duration_for(e, r->loopCount);
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
        ffb_effect_t *e = &g_effects[g_block_load.effectBlockIndex];
        memset(e, 0, sizeof(ffb_effect_t));
        e->state = FFB_STATE_ALLOCATED;
        e->gain  = 255;   /* full gain until Set Effect says otherwise */
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
    if (report_type == FFB_REPORT_TYPE_INPUT) {
        /* Some hosts issue GET_REPORT(Input) for the PID state on open. */
        if (report_id == FFB_RID_PID_STATE &&
            reqlen >= sizeof(ffb_pid_state_report_t)) {
            g_pid_state.status = FFB_STATUS_POWER
                | (g_device_paused     ? FFB_STATUS_PAUSED    : 0)
                | (g_actuators_enabled ? FFB_STATUS_ACTUATORS : 0);
            memcpy(buffer, &g_pid_state, sizeof(ffb_pid_state_report_t));
            return sizeof(ffb_pid_state_report_t);
        }
        return 0;
    }
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

/* All effect math works in the descriptor's native -10000..10000 units;
 * ffb_calculate() converts the summed force to ±32767 at the very end. */
#define FFB_UNIT_MAX  10000

/* Current envelope amplitude (0..10000). `sustain` is the effect's nominal
 * amplitude; attack ramps attackLevel→sustain, fade ramps sustain→fadeLevel
 * over the last fadeTime ms of the total playback time. */
static int32_t envelope_level(const ffb_effect_t *e, int32_t sustain) {
    uint32_t el = e->elapsedTime;
    /* 64-bit intermediates: attack/fade times are 32-bit ms values. */
    if (e->attackTime && el < e->attackTime) {
        return e->attackLevel + (int32_t)
               ((int64_t)(sustain - e->attackLevel) * el / e->attackTime);
    }
    if (e->fadeTime && e->totalDuration != FFB_TOTALDUR_INFINITE &&
        e->totalDuration > e->fadeTime &&
        el > e->totalDuration - e->fadeTime) {
        uint32_t remaining = e->totalDuration - el;
        return e->fadeLevel + (int32_t)
               ((int64_t)(sustain - e->fadeLevel) * remaining / e->fadeTime);
    }
    return sustain;
}

/* Envelope for signed force values (constant/ramp): modulate the magnitude,
 * keep the sign. */
static int32_t envelope_signed(const ffb_effect_t *e, int32_t value) {
    int32_t lvl = envelope_level(e, value < 0 ? -value : value);
    return (value < 0) ? -lvl : lvl;
}

static int32_t calc_constant(const ffb_effect_t *e) {
    return envelope_signed(e, e->magnitude) * e->gain / 255;
}

static int32_t calc_ramp(const ffb_effect_t *e) {
    int32_t val = e->startMagnitude;
    if (!FFB_DURATION_IS_INF(e->duration)) {
        /* Modulo so looped playback ramps again each iteration; 64-bit so
         * elapsed × Δmagnitude can't overflow. */
        uint32_t t = e->elapsedTime % e->duration;
        val += (int32_t)((int64_t)t *
               (e->endMagnitude - e->startMagnitude) / (int32_t)e->duration);
    }
    return envelope_signed(e, val) * e->gain / 255;
}

/* Waveform time within one period, with the phase offset applied.
 * Phase is 0..35999 in units of 0.01 degrees (descriptor unit exp -2). */
static uint32_t periodic_time(const ffb_effect_t *e) {
    uint32_t phase_t = (uint32_t)((uint64_t)e->phase * e->period / 36000u);
    return (e->elapsedTime + phase_t) % e->period;
}

static int32_t calc_square(const ffb_effect_t *e) {
    if (e->period == 0) return 0;
    int32_t lvl = envelope_level(e, e->magnitude);
    uint32_t t = periodic_time(e);
    int32_t f = e->offset + ((t < e->period / 2) ? lvl : -lvl);
    return f * e->gain / 255;
}

static int32_t calc_sine(const ffb_effect_t *e) {
    if (e->period == 0) return 0;
    int32_t lvl = envelope_level(e, e->magnitude);
    uint32_t t = periodic_time(e);
    float angle = 2.0f * 3.14159265f * (float)t / (float)e->period;
    int32_t f = e->offset + (int32_t)(sinf(angle) * (float)lvl);
    return f * e->gain / 255;
}

static int32_t calc_triangle(const ffb_effect_t *e) {
    if (e->period == 0) return 0;
    int32_t lvl = envelope_level(e, e->magnitude);
    uint32_t rem = periodic_time(e);
    int32_t hi = e->offset + lvl, lo = e->offset - lvl;
    float slope = (float)(hi - lo) * 2.0f / (float)e->period;
    float f = (rem < e->period / 2) ? (lo + slope * rem)
                                    : (hi - slope * (rem - e->period / 2));
    return (int32_t)f * e->gain / 255;
}

static int32_t calc_sawtooth_up(const ffb_effect_t *e) {
    if (e->period == 0) return 0;
    int32_t lvl = envelope_level(e, e->magnitude);
    uint32_t rem = periodic_time(e);
    int32_t hi = e->offset + lvl, lo = e->offset - lvl;
    float slope = (float)(hi - lo) / (float)e->period;
    return (int32_t)(lo + slope * rem) * e->gain / 255;
}

static int32_t calc_sawtooth_down(const ffb_effect_t *e) {
    if (e->period == 0) return 0;
    int32_t lvl = envelope_level(e, e->magnitude);
    uint32_t rem = periodic_time(e);
    int32_t hi = e->offset + lvl, lo = e->offset - lvl;
    float slope = (float)(hi - lo) / (float)e->period;
    return (int32_t)(hi - slope * rem) * e->gain / 255;
}

/* Condition force: spring/damper/inertia/friction share this.
 * metric is the relevant axis signal normalized to -10000..10000
 * (matching the PID descriptor's condition parameter range).
 * PID formula: f = coefficient * (metric - (cp ± deadband)) — no sign flip
 * on the negative side, or a centering spring turns into a one-way slam. */
static int32_t calc_condition(const ffb_effect_t *e, int32_t metric) {
    int32_t cp  = e->cpOffset;
    int32_t db  = e->deadBand;
    int32_t f   = 0;
    if (metric < cp - db) {
        f = (metric - (cp - db)) * (int32_t)e->negativeCoefficient / 10000;
    } else if (metric > cp + db) {
        f = (metric - (cp + db)) * (int32_t)e->positiveCoefficient / 10000;
    }
    f = clip32(f, -(int32_t)e->negativeSaturation, (int32_t)e->positiveSaturation);
    return f * e->gain / 255;
}

/* Direction handling for force effects on a single-axis wheel. The PID
 * Direction field is 0..255 = 0..360° clockwise from north; the X component
 * is sin(angle) (east = 64 → +X, west = 192 → -X). Games that encode the
 * sign in the magnitude instead send direction 0 (or leave Direction Enable
 * clear) — pass those through unscaled. */
static int32_t apply_direction(const ffb_effect_t *e, int32_t f) {
    if (!(e->enableAxis & FFB_DIRECTION_ENABLE) || e->directionX == 0)
        return f;
    float angle = (float)e->directionX * (2.0f * 3.14159265f / 256.0f);
    return (int32_t)((float)f * sinf(angle));
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
        if (e->totalDuration != FFB_TOTALDUR_INFINITE &&
            e->elapsedTime > e->totalDuration) {
            stop_effect(id);
            continue;
        }
        switch (e->effectType) {
            case FFB_EFFECT_CONSTANT:      force += apply_direction(e, calc_constant(e)); break;
            case FFB_EFFECT_RAMP:          force += apply_direction(e, calc_ramp(e)); break;
            case FFB_EFFECT_SQUARE:        force += apply_direction(e, calc_square(e)); break;
            case FFB_EFFECT_SINE:          force += apply_direction(e, calc_sine(e)); break;
            case FFB_EFFECT_TRIANGLE:      force += apply_direction(e, calc_triangle(e)); break;
            case FFB_EFFECT_SAWTOOTHUP:    force += apply_direction(e, calc_sawtooth_up(e)); break;
            case FFB_EFFECT_SAWTOOTHDOWN:  force += apply_direction(e, calc_sawtooth_down(e)); break;
            case FFB_EFFECT_SPRING:        force += calc_condition(e, m->position); break;
            case FFB_EFFECT_DAMPER:        force += calc_condition(e, m->velocity); break;
            case FFB_EFFECT_INERTIA:       force += calc_condition(e, m->acceleration); break;
            case FFB_EFFECT_FRICTION:      force += calc_condition(e, m->velocity); break;
            default: break;
        }
    }

    /* Sum is in ±10000 descriptor units: clip, apply device gain, then
     * convert to the engine's ±32767 output range. */
    force = clip32(force, -FFB_UNIT_MAX, FFB_UNIT_MAX);
    force = force * (int32_t)g_device_gain / 255;
    force = force * FFB_TORQUE_MAX / FFB_UNIT_MAX;
    ffb_output_torque((int16_t)force);
}

/* ============================================================ */
/* Input report                                                 */
/* ============================================================ */

uint16_t ffb_build_wheel_report(uint8_t *buffer, uint16_t reqlen,
                                uint32_t buttons,
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
