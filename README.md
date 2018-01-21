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

Application Programming API
---------------------------

**Toboot Sets the Watchdog Timer**.  Your program **will** reboot if the watchdog timer isn't cleared within a few tens of milliseconds.  This is to ensure the code returns to the bootloader if you accidentally do something like flash an MP3 file, or try to program the .ihex version.

A quick-and-dirty way to do this is to put the following at the start of your program:

````c++
*(uint32_t *)0x40088000UL = 0;
````

Of course, it's better to actually use a Watchdog driver and keep the watchdog fed normally.  But this will at least get you going.

More information on the Toboot API is available in [API.md](API.md).

Building
--------

Build the bootloader by entering the "toboot" directory and typing "make".