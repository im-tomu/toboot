Booster: the Toboot Updater
===========================

Toboot cannot update itself, because a partial flash would result in an unbootable system.

`Booster` is used to guide the installation of Toboot.  By appending a new version of Toboot to the end of Booster, we can create a program to update Toboot itself.

Usage
-----

First, compile booster.  Then, append the application header, and finally append the application itself.  This can be accomplished with `make-booster`.

````sh
cd toboot/
make
cd ../booster/
make
gcc make-booster.c -o make-booster
./make-booster ../toboot/toboot.bin toboot-booster.bin
````

The resulting `toboot-booster.bin` can be flashed with Toboot itself, or can be loaded using the legacy serial bootloader.

Design
------

Booster uses xxHash to verify the application is loaded correctly.  It also needs to know how many bytes to load.  To do this, it looks at the `booster_data` struct, which has the following format:

````c++
struct booster_data
{
    uint32_t payload_size;  // Number of bytes to flash
    uint32_t xxhash32;      // A hash of the entire program to be loaded
    uint32_t payload[0];    // The contents of the firmware to build
};
````

The `payload_size` value indicates the number of bytes to write.  Ideally it's a multiple of four.

The `xxhash32` is the result of a 32-bit xxHash operation.  The seed is defined in booster.h as `BOOSTER_SEED`, and is `0x68d9d190L`.

Finally, `payload` is an array of uint32 values.

The `struct booster_data` object is placed directly following the program image.  The `make-booster` program copies the contents of `booster.bin` to a new file, then appends `struct booster_data` to the end.  That way, all `booster` has to do is refer to the contents of the struct in order to program the data.

As a happy coincidence, if `struct booster_data` is not populated (i.e. if you just flash the raw `booster.bin` to a device), then `xxhash32` will not be valid and `booster.bin` will simply reset the device.