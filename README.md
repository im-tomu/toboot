Bootloader for [Tomu board](http://tomu.im)
===========================================

This repo contains Toboot and associated support files for the [EFM32HG Tomu board](https://github.com/im-tomu/tomu-hardware).

Upgrading Toboot
----------------

Toboot is unable to reflash itself.  This is to prevent partial updates from corrupting the firmware.  Instead, a support program is appended to the start of Toboot, and the entire thing is uploaded as one chunk.

`Booster` is the name of the program used to update Toboot itself.  The source code is located in the [booster/](booster) directory.

Use `make-booster` to wrap toboot.bin in a booster app, and flash the resulting image using dfu-util:

````sh
cd toboot/
make
cd ../booster/
make
gcc make-booster.c -o make-booster
./make-booster ../toboot/toboot.bin toboot-booster.bin
````

You can then flash the resulting `toboot-booster.bin` using dfu-util, or using the legacy serial uploader:

`dfu-util -d 1209:70b1 -D toboot-booster.bin`

Flashing onto a new Tomu
------------------------

Tomu's received after February 2017 should come with an earlier version of this bootloader installed.
If you got your Tomu at LCA2017 or 33C3, or building your own Tomu board you
will need to load the bootloader yourself.

The recommend way to load the bootloader onto a Tomu board is using a Raspberry Pi with
[OpenOCD](http://openocd.org/). Instructions for doing this can be found in the
[openocd](openocd) directory. You need OpenOCD **version 0.10.0 or later** to
have EFM32HG support.

Using
-----

**To enter Toboot manually, short out the outer two pads when applying power**.

Users normally won't see Toboot.  Instead, the installed application will run automatically.  Toboot will run if any of the following occurs:

1. There is no main application loaded.  This can happen if you've erased the flash, or if you've loaded an invalid binary.  The program's start address must be in flash, and the stack pointer must be in RAM.
1. The board has failed to finish booting three times in a row.
1. The magic value `0x74624346` is stored in the boot token area
1. The user shorts the two outer pads together when they apply power.
1. The config value doesn't have `TOBOOT_CONFIG_FLAG_POWERON_ENTER` set, and the board has just been powered on.

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

Build the bootloader by entering the `toboot/` directory and typing `make`.

Legacy Bootloader
-----------------

SiLabs AN0042 was the original bootloader.  It requires an IAR compiler to build, as well as custom drivers/software on the host device.  This bootloader is available in the 'an0042' branch, and has been removed from the master branch.  It is here for historical interest, and for compatibility with stock EFM32HG utilities.
