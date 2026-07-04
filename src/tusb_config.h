/*
 * tusb_config.h - TinyUSB configuration for FFB wheel
 */
#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Device stack only, bare-metal (no RTOS). RP2040 runs TinyUSB in poll mode.
 * CFG_TUSB_MCU and CFG_TUSB_OS are set by the Pico SDK build system. */
#define CFG_TUSB_DEBUG          0

#define CFG_TUD_ENABLED         1
#define CFG_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED

#define CFG_TUD_ENDPOINT0_SIZE  64

/* One HID interface (composite FFB wheel). No CDC/MSC. */
#define CFG_TUD_CDC             0
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             1
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          0

/* FFB reports are small (max ~14 bytes); 64 is plenty. Increase if you add
 * custom force data reports. */
#define CFG_TUD_HID_EP_BUFSIZE  64

#define CFG_TUSB_MEM_ALIGN      __attribute__((aligned(4)))

#ifdef __cplusplus
 }
#endif

#endif /* TUSB_CONFIG_H */
