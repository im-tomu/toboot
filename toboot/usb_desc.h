/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2013 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows 
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _usb_desc_h_
#define _usb_desc_h_

#include <stdint.h>
#include <stddef.h>
#include "dfu.h"

struct device_req {
    union {
        struct {
            uint8_t bmRequestType;
            uint8_t bRequest;
        };
        uint16_t wRequestAndType;
    };
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};

struct ctrl_data {
  uint8_t *addr;
  uint16_t len;
  uint8_t require_zlp;
};

struct usb_dev
{
    uint8_t configuration;
    uint8_t feature;
    uint8_t state;
    struct device_req dev_req;
    struct ctrl_data ctrl_data;
};
extern struct usb_dev *dev;

#define NUM_USB_BUFFERS           8
#define VENDOR_ID                 0x1d50    // OpenMoko
#define PRODUCT_ID                0x6082    // Assigned to Fadecandy project for DFU Bootloader
#define DEVICE_VER                0x0101    // Bootloader version
#define MANUFACTURER_NAME         {'x','o', 'b', 's'}
#define MANUFACTURER_NAME_LEN     4
#define PRODUCT_NAME              {'T','o','m','u',' ','B','o','o','t','l','o','a','d','e','r'}
#define PRODUCT_NAME_LEN          15
#define EP0_SIZE                  64
#define NUM_INTERFACE             1
#define CONFIG_DESC_SIZE          (9+9+9)

// Microsoft Compatible ID Feature Descriptor
#define MSFT_VENDOR_CODE    '~'     // Arbitrary, but should be printable ASCII
#define MSFT_WCID_LEN       40
extern uint8_t usb_microsoft_wcid[MSFT_WCID_LEN];

typedef struct {
    uint16_t  wValue;
    const uint8_t *addr;
    uint16_t  length;
} usb_descriptor_list_t;

extern const usb_descriptor_list_t usb_descriptor_list[];

enum FEATURE_SELECTOR
{
  FEATURE_ENDPOINT_HALT=0,
  FEATURE_DEVICE_REMOTE_WAKEUP=1
};

#endif