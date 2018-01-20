
AN0042 Bootloader
=================

This bootloader is a [slightly modified version](./an0042_efm32) of the stock
EFM32 AN0042 bootloader.

The changes are;

 * [Use internal oscillator rather than requiring an external Crystal (which doesn't exist on the Tomu).][1]
 * Always enter the bootloader, not needing the a pullup on the 'C' pin

[1]: https://www.silabs.com/documents/public/application-notes/AN0042.pdf

The bootloader will identify itself as follows when the `i` command is pressed;
```
BOOTLOADER version EFM32HG bootloader v1.a, Chip ID 24A622015669A7A1
```

The two important parts are <code>version EFM32**HG** bootloader v1.**a**</code>

Using
------

New firmware can be loaded using the `u` command.
Booting the loaded firmware can be done using `b`.

[For a full set of commands see the AN0042 documentation][2].

 [2]: https://www.silabs.com/documents/public/application-notes/AN0042.pdf

Building
-----------

The firmware requires IAR compiler which is not freely avaliable. Prebuilt
versions of the bootloader are available in the [prebuilt](prebuilt) directory.

[We would love someone to port the bootloader to GCC!](https://github.com/im-tomu/tomu-bootloader/issues/5)