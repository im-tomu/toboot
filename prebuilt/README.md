
# File formats

* `ihex` - Intel Hex file useful for tools like Keil ULINK2 or OpenOCD.
* `bin`  - Raw binary file can be flashed using OpenOCD, or attached to Booster.
* `elf`  - Debuggable versions suitable for playing with gdb, or loading with OpenOCD.

# Files

## `toboot`

Current version of toboot DFU-based bootloader.

## `toboot-boosted`

Self-installing version of Toboot.  Flash this using either dfu-util (Toboot) or xmodem (AN0042 bootloader) to move to the newest version of Toboot.