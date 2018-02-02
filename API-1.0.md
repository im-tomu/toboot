Toboot API v1.0
===============

Toboot allows for some interaction between your application and the Tomu itself.

Version
-------

This document describes v1.0 of the API, which is the default API used if your program doesn't declare compatibility with another API version.

Watchdog Timer
--------------

Toboot sets the watchdog timer.  You must disable it, or start feeding it right away.  A quick hack to disable the Watchdog timer is to write:

````c++
*(uint32_t *)0x40088000UL = 0;
````

Application Entrypoint
----------------------

Your application's stack pointer is expected to live at offset 0x4000, and the entrypoint is expected to be at offset 0x4004.  This is the same as the stock Silabs bootloader, and follows standard ARM Cortex M0 behavior.

Toboot is guaranteed to exist within the first 0x2000 bytes, and in fact future versions may even be under 0x1000 bytes.  This means that you can have code and data within the 0x2000-0x4000 region.  You can even use it as storage if you like.

By adopting this convention, we ensure compatibility with legacy Toboot code and bootloaders.

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

Toboot Configuration Values
---------------------------

The EFM32HG has 21 interrupt vectors, labeled Vector40, Vector44, Vector48, ..., Vector90.  This is not divisible by 4, and so your environment will also reserve Vector94, Vector98, and Vector9C.  Normally these point to a default exception handler, however because these vectors cannot be called we reuse them to act as Toboot configuration values.

Vector94 (located 94 bytes from the start of your program) is the Toboot Configuration Flag.  This configures how Toboot behaves.  It is a 32-bit word with the following format:

````c++
0xrrrrLLLL
````

*rrrr* are reserved values.  Set these to 0.

If *LLLL* is set to 0x70b0, then users cannot enter Toboot by shorting out the pads.  Use this to lock out firmware uploads, except programmatically.

Vector98 (located 98 bytes from the start of your program) is the Toboot App Flag.  This configures how your app is loaded.  It contains the following format:

````c++
0xrrSSKKKK
````

*rr* values are reserved.  Set these to 0.

The *KKKK* values are the key.  They must be set to 0x6fb0, otherwise the App Configuration Flag is ignored.

Set *SS* to the starting page number to load your program at.  For example, Toboot has this value set to 0x00006fb0, which causes it to be loaded beginning at page 0.  That way, Toboot can be used to reflash itself.

Page sizes are 1024 bytes.  To maintain compatibility with earlier programs, you may want to set this value to 0x00046fb0, which will explicitly cause programs to be loaded at offset 0x4000.