#ifndef BOARD_H_
#define BOARD_H_

// Board-specific settings

#define TOBOOT_BOARD_TOMU      0x23 // The original Tomu board
#define TOBOOT_BOARD_BAM_03    0xb3 // Buddy-Fox A-10C UFC Controller

#ifndef TOBOOT_BOARD
#    define TOBOOT_BOARD TOBOOT_BOARD_TOMU
#endif

#if TOBOOT_BOARD == TOBOOT_BOARD_TOMU
// Red LED at PB7
#    define LED0_PORT 1
#    define LED0_PIN 7
// Green LED at PA0
#    define LED1_PORT 0
#    define LED1_PIN 0
// Bootloader is forced by a short between CAP1A (PC1) and CAP1B (PE12)
#    define BUTTON_DRIVE_PORT 2
#    define BUTTON_DRIVE_PIN 1
#    define BUTTON_SENSE_PORT 4
#    define BUTTON_SENSE_PIN 12
// USB identifiers
#    define VENDOR_ID                 0x1209    // pid.codes
#    define PRODUCT_ID                0x70b1    // Assigned to Tomu project
#    define MANUFACTURER_NAME         u"Kosagi"
#    define PRODUCT_NAME              u"Tomu Bootloader (0) " GIT_VERSION
#    define REASON_OFFSET             17        // index of reason code in product name
// USB features
#    define ENABLE_WEBUSB
#    define LANDING_PAGE_URL          "dfu.tomu.im"
#    define ENABLE_WCID
#elif TOBOOT_BOARD == TOBOOT_BOARD_BAM_03
// Master Caution LED at PD7
#    define LED0_PORT 3
#    define LED0_PIN 7
// Master Caution button is between PB11 and PA2
#    define BUTTON_DRIVE_PORT 1
#    define BUTTON_DRIVE_PIN 11
#    define BUTTON_SENSE_PORT 0
#    define BUTTON_SENSE_PIN 2
// USB identifiers
#    define VENDOR_ID                 0x10c4    // Silicon Labs
#    define PRODUCT_ID                0x8c8b    // Assigned to Bamseco Oy
#    define MANUFACTURER_NAME         u"Bamseco Oy"
#    define PRODUCT_NAME              u"Buddy-Fox A-10C UFC Bootloader"
// USB features
#    define ENABLE_WEBUSB
#    define LANDING_PAGE_URL          "www.buddy-fox.com/fw-bam-03/"
#    define ENABLE_WCID
#else
#    error "Unknown board!"
#endif

#endif // BOARD_H_
