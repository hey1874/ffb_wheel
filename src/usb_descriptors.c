/*
 * usb_descriptors.c - USB device/config/string/HID-report descriptors
 *
 * VID/PID 0x1209/0xFFB0 match OpenFFBoard so games that register devices by
 * ID (Dirt/EA WRC device_defines.xml) recognise the wheel without edits.
 * HID report descriptor is the battle-tested VNWheel FFB descriptor.
 */
#include "tusb.h"
#include "bsp/board_api.h"        /* board_usb_get_serial */
#include "ffb_descriptors.h"   /* _hidReportDescriptor[] */
#include <string.h>            /* strlen */

/* ---- Device descriptor ---- */
static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x1209,
    .idProduct          = 0xFFB0,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

/* ---- HID report descriptor ---- */
uint8_t const *tud_hid_descriptor_report_cb(uint8_t itf) {
    (void)itf;
    return _hidReportDescriptor;
}

/* ---- Configuration descriptor ---- */
enum { ITF_NUM_HID, ITF_NUM_TOTAL };
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)

#define EPNUM_HID_OUT  0x01
#define EPNUM_HID_IN   0x81

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE,
                             sizeof(_hidReportDescriptor),
                             EPNUM_HID_OUT, EPNUM_HID_IN,
                             CFG_TUD_HID_EP_BUFSIZE, 1)
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

/* ---- String descriptors ---- */
enum { STRID_LANGID = 0, STRID_MANUFACTURER, STRID_PRODUCT, STRID_SERIAL };

static char const *string_desc_arr[] = {
    (const char[]){ 0x09, 0x04 },   /* English (0x0409) */
    "OpenFFB-Pico",                 /* Manufacturer */
    "FFB Wheel",                    /* Product */
    NULL                            /* Serial: board unique ID */
};

static uint16_t _desc_str[32 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    size_t chr_count;
    switch (index) {
        case STRID_LANGID:
            memcpy(&_desc_str[1], string_desc_arr[0], 2);
            chr_count = 1;
            break;
        case STRID_SERIAL:
            chr_count = board_usb_get_serial(_desc_str + 1, 32);
            break;
        default:
            if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
                return NULL;
            const char *str = string_desc_arr[index];
            if (!str) return NULL;
            chr_count = strlen(str);
            if (chr_count > 31) chr_count = 31;
            for (size_t i = 0; i < chr_count; i++)
                _desc_str[1 + i] = str[i];
            break;
    }
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
