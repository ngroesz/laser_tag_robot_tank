# Developer Documentation

If you want to write code for the Laser-Tag Robot Tank (hereafter, LTRT), this is the document for you.

## Requirements

### Arduino
Of course, you need the Arduino software. Make sure you have a recent version. At last update, the version I was using to develop was 1.8.12.

### Library Requirements:
Besides the core Arduino library, you will need some additional libraries. You can install these via the Arduino library manager.

 - IRMP (https://github.com/IRMP-org/IRMP)
 - PinChangeInterrupt (https://github.com/NicoHood/PinChangeInterrupt)
 - PVision (https://github.com/omwah/PVision)
 - VL53L0X (https://github.com/pololu/vl53l0x-arduino) *Note that I have been using cheap, knockoff ToF sensors sourced from Amazon.com but this library has worked fine for me.

### Optional Devtools:
  I do not like the Arduino IDE so I use arduino-cli to build and upload software to the tank.
  I have been using Ubuntu Linux to develop this software. If you use another OS or toolchain to
  develop software for LTRT, please feel invited to contribute to this documentation. I use the following in my dev toolchain:
 - Arduino-Makefile (https://github.com/sudar/Arduino-Makefile) I hate the Arduino IDE. I much prefer using vim to edit and Make to build/upload to the board.
In order to get this to work, I had to install the below arduino-cli and setup a configuration file for it (https://arduino.github.io/arduino-cli/1.1/configuration/) and install the arduino:avr platform]
 - arduino-cli (https://github.com/arduino/arduino-cli) Needed for above Arudino-Makefile

### Physical Requirements:
  You will, of course, need a Laser-Tag Robot Tank. You will also need a computer and an FTDI board in order to communicate with the Arduino chip on the LTRT circuit board.
You will also need an IR remote in order to communicate with the tank.

## Available Sketches

This repository comes with several sketches meant to demonstrate various aspects of the LTRT. For more information on the differenct sketches,
please see the [sketches](sketches.md) file.

Arduino has a requirement that the main ino file must match the directory name. I'm not sure that I have that requirement
exactly correct or whether there is a better workaround for that requirement. At any rate, what I do is use a symbolic
link called "laser_tag_robot_tank.ino" from the root of this repo and I point that link to whichever sketch I wish to
run at the time.

For instance, to use sketch called "motor_strain_test.ino":
```
ln -sf ./sketches/motor_strain_test.ino sketch.ino
```

## Code Documentation

I use Doxygen to automatically generate documentation. You can use a web browser to view the documentation
contained in this repo at [doxygen/html/index.html](doxygen/html/index.html). You will be most interested
in the Tank class, as the entrypoint for your code.

## Coding the LTRT

Finally, with all that out of the way, I wanted to give some advice on actually writing code for the LTRT.

### Practical Considerations

The LTRT is a robot that moves but it is easiest to code for it while sitting still. What I do is remove
the tracks for the LTRT and set it down on a desk. Then I plug in the FTDI and connect to my computer.
90% of my coding work is done in this way. All functions of the tank work but it will not drive off anywhere.
You can also try placing something under the tank in order to high-center it but I think it's easiest to
just remove the tracks.


### Coding A Real-Time System

If you have never worked with embedded systems or robots you may find the coding to be difficult at first.
I am an experienced software engineer but I found it difficult to wrap my mind around a real-time system.
The important thing to grasp is that your loop() function will execute many times per second. So, if you
want to turn right 90 degrees and then left 45 degrees, you CANNOT do something like:
```
tank.turn_right(90);
tank.turn_left(45);
```

Because this will, in effect, ignore the turn_right(90) call and merely execute turn_left(45).

Instead, you must use some combination of timers, state variables, and callbacks in order to keep track of
what is happening and make the robot do what you want.

#### Coding Without Delay

The delay() function pauses all execution on the Arduino. You must NEVER use this function in the main code
of your robot. It is okay to use it the setup() function. Using delay() will cause the rest of the robot to
not work correctly, as it relies on the tank.loop() function being called continuously.

Instead you must use variables to keep track of time and state, such that you can allow some specified amount
of time to pass between actions. You can do a web-search for "arduino coding without delay" for a lot more
information on this subject.

### Your Bot

In order to create your own robot, you will create a new sketch and you will use the Tank class in order to
control the robot. If you want, you can use the dumb_bot.ino sketch as a starting point. This sketch does
all the appropriate setup and initialization but does not contain any actual logic.

You will delare a global object:
```
Tank tank;
```

In your setup function, you need to initialize the tank:
```
void setup()
{
  // your own setup here
  tank.initialize();
}
```

Finally, it is critical that you call the tank loop() function from your own loop():
```
void loop()
{
  // your logic goes here
  tank.loop();
}
```

