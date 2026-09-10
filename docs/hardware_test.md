# Hardware Test Sketch

## Overview

The hardware_test.ino sketch is a good way to see what the tank is capable of
and to ensure that all of the tank's functions are in order.

You will need an IR remote to step the tank through the various modes. In order
to test the shooting and IR-tracking functions of the tank you will need an IR
source and IR receiver. Check the [Tank
Target](https://github.com/ngroesz/pcb_design/tree/master/tank_target) project
for an example of this.

The only remaining function of the tank that cannot be tested using a
combination of the IR remote and a Tank Target is the IR beacon. You can test
whether the beacon is illuminated using a digital camera, which will display
infrared light. Using your cell phone camera, look at the top of the lens that
is mounted on top of the turret mast. A small, purple-ish light should be
visible.

## Instructions

Refer to the Hardware Test Modes document linked below.

First press a number button on the IR remote in order to select a particular
mode. Then use other buttons (see Mode Details) to exercise the various
functions of the tank within the mode.


[Hardware Test Modes](hardware_test_modes.md)
