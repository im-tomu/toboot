
# File formats

* `ihex` - Intel Hex file useful for tools like Keil ULINK2 or OpenOCD.
* `bin`  - Raw binary file can be flashed using OpenOCD.
* `elf`  - Debuggable versions suitable for playing with gdb, or loading with OpenOCD.

# Files

## `bootld_unconditional`

File containing the
[unconditional version of the bootloader](https://github.com/im-tomu/tomu-bootloader/tree/master/an0042_efm32).

## `toboot`

Current version of toboot DFU-based bootloader.