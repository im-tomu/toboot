## OpenOCD Flashing ##

In order to make flashing `tomu` boards more accessible, this directory
contains information, scripts and configurations to allow flashing of the
`tomu` using a Raspberry Pi and [two-wire debugging][swd]. While these
configurations are specifically for a Raspberry Pi, the setup should be similar
for any system that allows you to use GPIO pins and can run GNU/Linux (and
OpenOCD of course).

[swd]: https://en.wikipedia.org/wiki/JTAG#Serial_Wire_Debug

### Setup ###

I'm currently working on a set of scripts to make this much simpler, but at the
moment setting up your Raspberry Pi has to be done manually. This includes
compiling OpenOCD from source. The tutorial I used can be found
[here][adafruit-tut], though setting up networking is out of scope for that
tutorial. Follow the tutorial until the step after installing OpenOCD, at which
point you should run this command:

```bash
sudo cp raspberrypi*-native.cfg /usr/local/share/openocd/scripts/interface/
```

This installs all of the necessary configurations to use two-wire debugging to
flash a `tomu`. After installing OpenOCD you can flash your `tomu` by placing
the firmware you wish to flash in the current directory with the name
`program.bin`, and running `sudo openocd -f tomu.conf`. You can also do the
configuration manually if you wish.

[adafruit-tut]: https://learn.adafruit.com/programming-microcontrollers-using-openocd-on-raspberry-pi

### Wiring ###

Contrary to its name, two-wire debugging (or more accurately "Serial Wire
Debug") uses four wires: `vcc`, `swdio`, `swdclk`, and `ground`. Looking at the
image below of the back of the `tomu`, that is the left-to-right order of
solder pads. The colour coding is `red=vcc`, `blue=swdio`, `green=swdclk`,
`square=ground`.

![back of the tomu](tomu-back.png)

It is not necessary to solder anything to your `tomu` in order to flash it. If
you have a steady hand you can just hold the wires in place for the couple of
seconds it takes to flash a board. I super-glued some breadboard wires together
to ensure the spacing is constant.

The pins used by the configuration files for the Raspberry Pi in this directory
for two-wire debugging are pins `24` and `25` for `swdio` and `swclk`
respectively. As far as I can tell, Raspberry Pi pin numbering **is** backwards
compatible, so if your Raspberry Pi has less pins than in the diagram below
just count the pins from the left. In the diagram, `vcc` is orange and `ground`
is black. Make sure you get the ordering of the pins correct or you'll end up
restarting your board (due to a short-circuit).

![raspberry pi GPIO pin out](raspberry-pi-pinout.png)

If you're flashing a `tomu` with a different board you'll need to figure out
the right pins to use on your own.
