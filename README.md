# Laser Tag Robot Tank

Laser Tag Robot Tank is a toy tank designed to fight autonomous laser-tag.
This repository stores the code that runs the tank. For the physical 
manifestation of the tank, see the Links section below.

## Quickstart

Assuming you have all the software requirements installed, and you have a cable
connected to a tank, from this repo you can do:

```
ln -s sketches/hardware_test.ino ./laser_tag_robot_tank.iso arduino-cli make
arduino-cli make upload
```

The above will upload the sketch "hardware_test.ino" to the tank. Then you can
use an IR remote to test out the various functions of the tank. See the
"hardware_test.ino" section in the [sketches](docs/sketches.md#harware_test.ino)
documentation file.

If the above does not work for you, please see more information in the
[development](docs/development.md) file.

## Links

This repository is for the code to run the tank. To build a Laser Tag Robot
Tank from scratch you will need to examine:

[3D Files](https://github.com/ngroesz/3d_printing/tree/master/tank) - The files
used to 3D print the physical parts of the tank.

[PCB Files](https://github.com/ngroesz/pcb_design/tree/master/tank) - Files used
to make printed circuit boards for the tank.

## More Details

Laser Tag Robot Tank is designed to be:
* Cheap (Well, cheap-ish. I estimate the build cost to be around $100. Many
parts are 3D-printed but other parts are off-the-shelf and somewhat expensive,
such as gearboxes and Hall-effect sensors.)
* Autonomous - The idea is that you program the tank and then send it into
battle to fight tanks that others have progammed. To faciliate this, the tank
has wheel encoders, bumpers, an IR camera, and a distance sensor. The library in
this repo is designed to abstract some of the more fiddly bookkeeping bits so
that you can focus on your bot's fighting skills.
* Easy for anyone to build. This is relative, of course. But the idea is that
anyone with a 3D printer, soldering tools, a source for circuit boards, an old
Wiimote, and some basic electrics knowledge can build their own Laser Tag Robot
Tank.

## Documentation

Check out the [documentation README](docs/README.md) for a list of
documentation.

