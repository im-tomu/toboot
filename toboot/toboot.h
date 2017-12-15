#ifndef TOBOOT_H_
#define TOBOOT_H_

#include <stdint.h>

#define BOOTLOADER_ENTER_TOKEN 0x74624346

#define PACKED __attribute__((packed))

// This describes the structure that allows the OS to communicate
// with the bootloader.  It keeps track of how many times we've
// tried booting, as well as a magic value that tells us to enter
// the bootloader instead of booting the app.
// It also keeps track of the board model.
struct boot_token
{
  uint32_t magic;
  uint8_t  boot_count;
  uint8_t  board_model;
  uint16_t reserved;
} PACKED;
__attribute__((section("boot_token"))) extern struct boot_token boot_token;

enum bootloader_reason
{
  NOT_ENTERING_BOOTLOADER = 0,
  BOOT_TOKEN_PRESENT = 1,
  BOOT_FAILED_TOO_MANY_TIMES = 2,
  NO_PROGRAM_PRESENT = 3,
  BUTTON_HELD_DOWN = 4,
};

extern enum bootloader_reason bootloader_reason;

#endif /* TOBOOT_H_ */