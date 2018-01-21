Bootloader for [Tomu board](http://tomu.im)
===========================================

This repo contains a collection of bootloaders for the [EFM32HG Tomu board](https://github.com/im-tomu/tomu-hardware).

The original bootloader is located under an0042, and contains code from the Silabs appnote describing the inbuilt bootloader that ships on some EFM32HG parts.  It is here for historical interest, and for compatibility with stock EFM32HG utilities.

The current preferred bootloader is Toboot, which implements a DFU device.

Flashing onto Tomu
-------------------

Tomu's received after February 2017 should come with this bootloader installed.
If you got your Tomu at LCA2017 or 33C3, or building your own Tomu board you
will need to load the bootloader yourself.

The recommend way to load the bootloader onto a Tomu board is using a Raspberry Pi with
[OpenOCD](http://openocd.org/). Instructions for doing this can be found in the
[openocd](openocd) directory. You need OpenOCD **version 0.10.0 or later** to
have EFM32HG support.

Using
-----

Users normally won't see Toboot.  Instead, the installed application will run automatically.  Toboot will run if any of the following occurs:

1. There is no main application loaded.  This can happen if you've erased the flash, or if you've loaded an invalid binary.  The program's start address must be in flash, and the stack pointer must be in RAM.
1. The board has failed to finish booting three times in a row.
1. The magic value "0x74624346" is stored in the boot token area
1. The user shorts the two outer pads together when they apply power.

You can tell you're in Toboot because the red and green LEDs alternate four times per second.  In this mode, you can use dfu-util (or a [web version](https://devanlai.github.io/webdfu/dfu-util/)) to upload a new firmware.  Tomu will reboot once it has completed.

When you're in the bootloader, the lights will flash like this:

![Toboot Pattern](media/toboot-mode.gif?raw=true "Toboot Pattern")

Application Support
-------------------

Toboot sets the watchdog timer and increments the boot count when it runs.  That way it can tell if your application isn't behaving properly.  The following struct can be used to interact with the bootloader:

````
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
````

The file "tomu.ld" in this repo can be used to place your program at the correct location (offset 0x2000), as well as setting up the boot token.

On boot, you should reset the watchdog timer and clear the boot count.  Something like this should do nicely:

````
  WDOG->CTRL = 0;
  boot_token.boot_count = 0;
````

If you want to boot into the bootloader, set "magic" to the magic value and reboot:

````
  boot_token.magic = 0x74624346;
  SCB->AIRCR = 0x05FA0004; // Use AIRCR method of reset
````

Users can upload code using the bootloader, and can enter the bootloader by shorting out the two outer pins while applying power.  If you would like to disable this feature, set offset 0x94 of your application (i.e. the address of Vector94) to be 0xXXXX70b0.  This will disable the pin check, but you can still enter the bootloader by failing to boot, or by setting the magic value.

Building
--------

Build the bootloader by entering the "toboot" directory and typing "make".