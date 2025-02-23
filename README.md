# laser_tag_robot_tank

Arduino code to create a laser-shooting robotic tank.

## Software Requirements

### Arduino
Of course, you need the Arduino software. Make sure you have a recent version. At last update, the version I was using to develop was 1.8.12.

### Library requirements:
You can install these via the Arduino library manager.

 - PinChangeInterrupt (https://github.com/NicoHood/PinChangeInterrupt)
 - IRMP (https://github.com/IRMP-org/IRMP)
 - PVision (https://github.com/omwah/PVision)

### Optional
 - Arduino-Makefile (https://github.com/sudar/Arduino-Makefile) I hate the Arduino IDE. I much prefer using vim to edit and Make to build/upload to the board.
	[In order to get this to work, I had to install the below arduino-cli and setup a configuration file for it (https://arduino.github.io/arduino-cli/1.1/configuration/) and install the arduino:avr platform]
 - arduino-cli (https://github.com/arduino/arduino-cli) Needed for above Arudino-Makefile

## Various Sketches

The primary sketch is located at sketches/laser_tag_robot_tank.ino. Arduino-Makefile does not have support for multiple targets. Therefore, in order to use
a different sketch you will need to change the sketch.ino symbolic link: `ln -sf ./sketches/motor_strain_test.ino sketch.ino`
