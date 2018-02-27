Toboot API V2.0
===============

Toboot allows for some interaction between your program and the running bootloader.  This document describes V2.0 of the API, which is defined by a magic value at the start of the header.

Version Differences
-------------------

There are several differences between V2.0 of the API and V1.0.  Notable differences include:

* Full program header, rather than reusing unused interrupt vectors
* Bitfield indicating sectors to erase when replacing a program
* Generational counter, allowing program to be placed anywhere in RAM

Program Header
--------------

Your application binary should have a program header located just after the interrupt vectors, at offset 0x94 from the start of your program.  This header should have the following structure:

````c++
struct toboot_configuration {
    uint32_t magic;         // Set to 0x907070b2
    uint8_t  start;         // The starting page for your program
    uint8_t  config;        // Configuration flags.  Defined below.
    uint16_t reserved_gen;  // Reserved value.  Toboot will set this to the generational counter.
    uint32_t lock_entry;    // Set to 0x18349420 to prevent the user from entering Toboot manually.
    uint32_t erase_mask_lo; // A bitmask of sectors to erase when updating the program
    uint32_t erase_mask_hi; // A bitmask of sectors to erase when updating the program
    uint32_t reserved_hash; // A hash of the header.  Calculated by Toboot.
};

/* Toboot V1.0 leaves IRQs enabled, mimicking the behavior of
 * AN0042.  Toboot V2.0 makes this configurable by adding a
 * configuration flag.
 */
#define TOBOOT_CONFIG_FLAG_ENABLE_IRQ_MASK  0x01
#define TOBOOT_CONFIG_FLAG_ENABLE_IRQ_SHIFT 0
#define TOBOOT_CONFIG_FLAG_ENABLE_IRQ       (1 << 0)
#define TOBOOT_CONFIG_FLAG_DISABLE_IRQ      (0 << 0)

/* When running a normal program, you won't want Toboot to run.
 * However, when developing new software it is handy to have
 * Toboot run at poweron.  Set this flag to enter Toboot whenever
 * the system has powered on for the first time.
 */
#define TOBOOT_CONFIG_FLAG_POWERON_ENTER_MASK   0x02
#define TOBOOT_CONFIG_FLAG_POWERON_ENTER_SHIFT  1
#define TOBOOT_CONFIG_FLAG_POWERON_ENTER        (1 << 1)
````

Toboot Magic
------------

The first block of your program should contain the value 0x907070b2 at offset 0x94.  This tells the Toboot loader to use V2.0 for loading.  If the value 0xXXXX70b0 is found, then V1.0 is used.

Start Sector
-------

The starting sector indicates where in RAM the first block will be loaded. This value should be located somewhere after Toboot.  It is illegal to load the program into the region populated by Toboot itself, even if you are updating Toboot.  To perform an update, use a separate program to do the updating.

Sectors will be erased before they are written.

Config Flags
------------

The only config flag present is enable_irq.  Set this bit to 1 to leave IRQs enabled when jumping to your target program.

Lock Entry
----------

You may want to prevent users from entering Toboot.  Be very careful when doing this, because it can cause a device to become unusable.

To prevent users from entering Toboot by shorting out pads when applying power, set lock_entry to 0x18349420.

It is still possible to enter Toboot due to a failed boot, or by setting the magic value.

Erase Mask
----------

The two bitmasks *erase_mask_lo* and *erase_mask_hi* define sectors that must be erased prior to programming.  If a bit is '1', then that sector will be erased, unless it is a sector used by Toboot.

If a bit is 0, then the specified sector might not be erased.  However, if the loaded program requires a sector to be programmed, then that sector will first be erased anyway.

To erase all of flash (except Toboot), set this value to 0xffffffff 0xffffffff.  To erase only the minimum number of sectors, set this value to 0x00000000 0x00000000.

You should use this to ensure that sensitive data stores are erased before replacing firmware.  For example, secret keys.

Generational Counter
--------------------

This counter keeps track of the version of the software present on Tomu.  The first program will have a generation of "1", and every subsequent program will have a generation of 1 more.

Toboot will launch the program that has the highest generational counter located within a valid Toboot header with a hash that matches.

For example, if offset 0x2000 contains a valid header and has a generational counter of 2, and offset 0x3000 has a valid header and has a generational counter of 3, then Toboot will run the program at offset 0x3000.

This generational counter is computed automatically by looking at all programs on the system and adding 1 to the result.  This result is stored in the *reserved_gen* field.

Hash
----

Toboot currently uses xxHash to calculate the hash of the header.  A program must have a valid Magic and a valid hash to be considered.  Programs with an invalid hash are not considered.

Watchdog Timer
--------------

Toboot sets the watchdog timer.  You must disable it, or start feeding it right away.  A quick hack to disable the Watchdog timer is to write:

````c++
*(uint32_t *)0x40088000UL = 0;
````

Application Entrypoint
----------------------

Your application's stack pointer is expected to live at offset 0x0 from the start of your program, and the entrypoint is expected to be at offset 0x4.  This is followed by the standard interrupt vector table.

Boot Token
----------

The first 8 bytes of memory (offset 0x20000000 - 0x20000008) are reserved for a Toboot token.  The boot token has the following structure:

````c++
struct boot_token {
    // Set this to 0x74624346 and reboot to enter bootloader
    uint32_t magic;

    // Set this to 0 when your program starts.
    uint8_t  boot_count;

    // The bootloader should set this to 0x23 for Tomu.
    uint8_t  board_model;

    // Unused.
    uint16_t reserved;
};
````

The value board\_model is defined to be 0x23.  The boot\_count value describes the number of times the board has rebooted.  If the board fails to reboot three times in a row, it will enter Toboot automatically.

The *magic* value allows you to enter Toboot programmatically.  Set this value to 0x74624346 and reboot.