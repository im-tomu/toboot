#ifndef BOARD_H_
#define BOARD_H_

// Board-specific settings

#define TOBOOT_BOARD_TOMU      0x23 // The original Tomu board

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
#else
#    error "Unknown board!"
#endif

#endif // BOARD_H_
