/*
 * ffb_types.h - FFB report data structures and constants
 *
 * Based on the PID 1.0 (Physical Input Device) USB HID class spec.
 * Report ID scheme matches the VNWheel descriptor (ffb_descriptors.h):
 *   Output and Feature reports share IDs 5/6/7, distinguished by report_type.
 * Structs omit the reportId byte (TinyUSB extracts it before invoking the
 * set/get_report callbacks — see hid_device.c lines 317-321 and 293-296).
 */
#ifndef FFB_TYPES_H
#define FFB_TYPES_H

#include <stdint.h>

/* ---- HID report types (matches TinyUSB hid_report_type_t enum) ---- */
#define FFB_REPORT_TYPE_INPUT    1
#define FFB_REPORT_TYPE_OUTPUT   2
#define FFB_REPORT_TYPE_FEATURE  3

/* ---- Effect type IDs (1-based enum order in the descriptor) ---- */
/* Matches PID Usage order: 0x26,0x27,0x30,0x31,0x32,0x33,0x34,0x40,0x41,0x42,0x43 */
#define FFB_EFFECT_NONE          0x00
#define FFB_EFFECT_CONSTANT      0x01
#define FFB_EFFECT_RAMP          0x02
#define FFB_EFFECT_SQUARE        0x03
#define FFB_EFFECT_SINE          0x04
#define FFB_EFFECT_TRIANGLE      0x05
#define FFB_EFFECT_SAWTOOTHUP    0x06  /* Usage 0x33 */
#define FFB_EFFECT_SAWTOOTHDOWN  0x07  /* Usage 0x34 */
#define FFB_EFFECT_SPRING        0x08
#define FFB_EFFECT_DAMPER        0x09
#define FFB_EFFECT_INERTIA       0x0A
#define FFB_EFFECT_FRICTION      0x0B
#define FFB_EFFECT_CUSTOM        0x0C

/* ---- Report IDs ---- */
/* Input */
#define FFB_RID_WHEEL            0x01
#define FFB_RID_PID_STATE        0x02
/* Output */
#define FFB_RID_SET_EFFECT       0x01
#define FFB_RID_SET_ENVELOPE     0x02
#define FFB_RID_SET_CONDITION    0x03
#define FFB_RID_SET_PERIODIC     0x04
#define FFB_RID_SET_CONSTANT     0x05
#define FFB_RID_SET_RAMP         0x06
#define FFB_RID_CUSTOM_DATA      0x07
#define FFB_RID_DOWNLOAD_SAMPLE  0x08
#define FFB_RID_EFFECT_OP        0x0A
#define FFB_RID_BLOCK_FREE       0x0B
#define FFB_RID_DEVICE_CTRL      0x0C
#define FFB_RID_DEVICE_GAIN      0x0D
#define FFB_RID_SET_CUSTOM       0x0E
/* Feature (SET: create new effect; GET: block load / pool) */
#define FFB_RID_CREATE_EFFECT    0x05
#define FFB_RID_BLOCK_LOAD       0x06
#define FFB_RID_PID_POOL         0x07

/* ---- Limits ---- */
#define FFB_MAX_EFFECTS          40   /* descriptor declares Logical Maximum 0x28 */
#define FFB_DURATION_INFINITE    0x7FFF

/* ---- Effect state flags ---- */
#define FFB_STATE_FREE           0x00
#define FFB_STATE_ALLOCATED      0x01
#define FFB_STATE_PLAYING        0x02

/* ---- Axis enable bits ---- */
#define FFB_AXIS_X               0x01
#define FFB_AXIS_Y               0x02
#define FFB_DIRECTION_ENABLE     0x04

/* ---- Device control commands ---- */
#define FFB_DC_ENABLE_ACTUATORS  0x01
#define FFB_DC_DISABLE_ACTUATORS 0x02
#define FFB_DC_STOP              0x03
#define FFB_DC_RESET             0x04
#define FFB_DC_PAUSE             0x05
#define FFB_DC_CONTINUE          0x06

/* PID state report status bits */
#define FFB_STATUS_PAUSED        0x01
#define FFB_STATUS_ACTUATORS     0x02
#define FFB_STATUS_SAFETY        0x04
#define FFB_STATUS_OVERRIDE      0x08
#define FFB_STATUS_POWER         0x10

/* ---- Input report structures (no reportId byte) ---- */
typedef struct __attribute__((packed)) {
    uint8_t  buttons;
    int16_t  xAxis;
    int16_t  yAxis;
    int16_t  zAxis;
    int16_t  rxAxis;
    int16_t  ryAxis;
    int16_t  rzAxis;
} ffb_wheel_report_t;

typedef struct __attribute__((packed)) {
    uint8_t status;           /* bitfield */
    uint8_t effectBlockIndex; /* bit7 = playing, bit0..6 = id */
} ffb_pid_state_report_t;

/* ---- Output report structures (no reportId byte) ---- */
typedef struct __attribute__((packed)) {
    uint8_t  effectBlockIndex;
    uint8_t  effectType;
    uint16_t duration;
    uint16_t triggerRepeatInterval;
    uint16_t samplePeriod;
    uint8_t  gain;
    uint8_t  triggerButton;
    uint8_t  enableAxis;
    uint8_t  directionX;
    uint8_t  directionY;
} ffb_set_effect_t;

typedef struct __attribute__((packed)) {
    uint8_t  effectBlockIndex;
    uint16_t attackLevel;
    uint16_t fadeLevel;
    uint16_t attackTime;
    uint16_t fadeTime;
} ffb_set_envelope_t;

typedef struct __attribute__((packed)) {
    uint8_t  effectBlockIndex;
    uint8_t  parameterBlockOffset;
    int16_t  cpOffset;
    int16_t  positiveCoefficient;
    int16_t  negativeCoefficient;
    uint16_t positiveSaturation;
    uint16_t negativeSaturation;
    uint16_t deadBand;
} ffb_set_condition_t;

typedef struct __attribute__((packed)) {
    uint8_t  effectBlockIndex;
    uint16_t magnitude;
    int16_t  offset;
    uint16_t phase;
    uint16_t period;
} ffb_set_periodic_t;

typedef struct __attribute__((packed)) {
    uint8_t  effectBlockIndex;
    int16_t  magnitude;
} ffb_set_constant_t;

typedef struct __attribute__((packed)) {
    uint8_t  effectBlockIndex;
    int16_t  startMagnitude;
    int16_t  endMagnitude;
} ffb_set_ramp_t;

typedef struct __attribute__((packed)) {
    uint8_t  effectBlockIndex;
    uint8_t  operation;   /* 1=Start, 2=StartSolo, 3=Stop */
    uint8_t  loopCount;
} ffb_effect_op_t;

typedef struct __attribute__((packed)) {
    uint8_t  effectBlockIndex;
} ffb_block_free_t;

typedef struct __attribute__((packed)) {
    uint8_t  control;
} ffb_device_control_t;

typedef struct __attribute__((packed)) {
    uint8_t  gain;
} ffb_device_gain_t;

/* ---- Feature report structures (no reportId byte) ---- */
typedef struct __attribute__((packed)) {
    uint8_t  effectType;
    uint16_t byteCount;
} ffb_create_effect_t;

typedef struct __attribute__((packed)) {
    uint8_t  effectBlockIndex;
    uint8_t  loadStatus;     /* 1=Success, 2=Full, 3=Error */
    uint16_t ramPoolAvailable;
} ffb_block_load_t;

typedef struct __attribute__((packed)) {
    uint16_t ramPoolSize;
    uint8_t  maxSimultaneousEffects;
    uint8_t  memoryManagement; /* bit0=DeviceManagedPool, bit1=SharedParamBlocks */
} ffb_pid_pool_t;

/* ---- Internal effect state ---- */
typedef struct {
    volatile uint8_t state;     /* FFB_STATE_* */
    uint8_t  effectType;
    int16_t  offset;            /* periodic offset / condition cpOffset */
    uint8_t  gain;
    int16_t  attackLevel, fadeLevel;
    int16_t  magnitude;         /* constant / periodic magnitude */
    int16_t  startMagnitude, endMagnitude; /* ramp */
    uint16_t period;
    uint16_t phase;
    uint16_t duration;
    uint16_t attackTime, fadeTime;
    uint32_t startTime;         /* ms, from to_ms_since_boot */
    uint32_t elapsedTime;       /* ms since start */
    /* condition params */
    int16_t  cpOffset;
    int16_t  positiveCoefficient;
    int16_t  negativeCoefficient;
    uint16_t positiveSaturation;
    uint16_t negativeSaturation;
    uint16_t deadBand;
} ffb_effect_t;

#endif /* FFB_TYPES_H */
