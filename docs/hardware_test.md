# Hardware Test Sketch

## Overview

The hardware_test.ino sketch is a good way to see what the tank is capable of
and to ensure that all of the tank's functions are in order.

You will need an IR remote to step the tank through the various modes. In order
to test the shooting and IR-tracking functions of the tank you will need an IR
source and IR receiver. Check the [Tank
Trainer](https://github.com/ngroesz/pcb_design/tree/master/tank_trainer) project
for an example of this.

The only remaining function of the tank that cannot be tested using a
combination of the IR remote and a Tank Trainer is the IR beacon. You can test
whether the beacon is illuminated using a digital camera, which will display
infrared light. Using your cell phone camera, look at the top of the lens that
is mounted on top of the turret mast. A small, purple-ish light should be
visible.

## Instructions

First press a number button on the IR remote in order to select a particular
mode. Then use other buttons (see Mode Details) to exercise the various
functions of the tank within the mode.

| Remote Button | Mode | Tests | Mode Details |
| --- | --- | --- | --- |
| 1 | Drive | Drive motors | Up: drive forward <br /> Down: drive backwards <br /> Right: turn right <br /> Left: turn left <br /> OK: Stop|
| 2 | Drive Measured | Drive motors, wheel encoders |  Up: drive forward 10 cm <br /> Down: drive backwards 10 cm<br /> Right: turn right 90 degrees <br /> Left: turn left 90 degrees<br /> OK: Stop|
| 3 | Turret | Turret motor | Left: Turret left <br /> Right: Turret right <br /> OK: Turret stop|
| 4 | Turret Measured | Turret motor, turret encoder | Left: Turret left 90 degrees <br /> Right: Turret right 90 degrees<br /> OK: Turret stop|
| 5 | Turret Calibration | Turret motor, turret encoder | OK: Turret will turn until it faces the 0-degree position|
| 6 | Distance | Distance sensor | Hold your hand or some object in front of the turret. You should see the tank's red LEDs illuminate based on distance from the turret. <br /> More than 750 mm: LED 1  <br /> Between 250 and 750 mm: LED 1 and 2 <br /> Less than 250 mm: LED 1, 2, and 3 |
| 7 | Camera | IR camera, turret motor | Hold an IR LED in front of the tank turret. The turret should track the IR light source back and forth as it is moved. |
| 8 | LED | Addressable LEDs | Tank has one green LED that is illuminated whenever the tank is switched on. Three red LEDs are addressable. <br /> Up: All LEDs on <br /> Left: LED 1 on <br /> Down: LED 2 on <br /> Right: LED 3 on <br /> OK: All LEDs off |
| 9 | Speaker | Speaker | Press OK and the tank should make a noise |
