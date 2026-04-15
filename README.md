# Laser Tag Robot Tank

## What This Is

This is more or less what is described in the title. An autonomous, laser-tag playing tank.
This repository stores the code that runs the tank. For the physical manifestation of the
tank, see the Links section below.

## Quickstart

Assuming you have all the software requirements installed, and you have a cable connected
to a tank, from this repo you can do:

```
ln -s sketches/hardware_test.ino ./laser_tag_robot_tank.iso
arduino-cli make
arduino-cli make upload
```

The above will upload the sketch "hardware_test.ino" to the tank. Then you can use an IR
remote to test out the various functions of the tank. See the "hardware_test.ino" section
in the [sketches](docs/sketches.md#harware_test.ino) documentation file.

If the above does not work for you, please see more information in the [development](docs/development.md)
file.

## Links

This repository is just for the code to run the tank. To build a Laser Tag Robot Tank from scratch you
will need to examine:

[3D Files](https://github.com/ngroesz/3d_printing/tree/master/tank) - The files used to 3D print the
physical parts of the tank.

[PCB Files](https://github.com/ngroesz/pcb_design/tree/master/tank) - Files used to make printed
circuit boards for the tank.
