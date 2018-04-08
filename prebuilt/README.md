
# File formats

* `ihex` - Intel Hex file useful for tools like Keil ULINK2 or OpenOCD.
* `bin`  - Raw binary file can be flashed using OpenOCD, or attached to Booster.
* `elf`  - Debuggable versions suitable for playing with gdb, or loading with OpenOCD.
* `dfu`  - Raw binary file with DFU suffix

# Files

## `toboot`

Current version of toboot DFU-based bootloader.

## `toboot-boosted`

Self-installing version of Toboot.  Flash this using either dfu-util (Toboot) or xmodem (AN0042 bootloader) to move to the newest version of Toboot.

The DFU version was created with the following command:

    cp toboot-boosted.bin toboot-boosted.dfu
    dfu-suffix --pid 0x70b1 --vid 0x1209 --add ./toboot-boosted.dfu
