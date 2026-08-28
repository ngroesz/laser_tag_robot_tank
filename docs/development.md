# Developer Documentation

If you want to write code for the Laser-Tag Robot Tank (hereafter, LTRT), this is the document for you.

## Quickstart

Check out the [quickstart](quickstart.md) file for the quickest possible way to get your robot tank to do something interesting.

## Requirements

### Arduino
Of course, you need the Arduino software. Make sure you have a recent version. At last update, the version I was using to develop was 1.8.12.

### Library Requirements:
Besides the core Arduino library, you will need some additional libraries. You can install these via the Arduino library manager.

As of last writing, here are the versions according to my arduino-cli output:
Used library       Version
IRremote           4.4.2
PinChangeInterrupt 1.2.8
PVision            0.0.4
VL53L0X            1.3.1
Wire               1.0

Here are where to find those libraries:
 - IRMP (https://github.com/IRMP-org/IRMP)
 - PinChangeInterrupt (https://github.com/NicoHood/PinChangeInterrupt)
 - PVision (https://github.com/omwah/PVision)
 - VL53L0X (https://github.com/pololu/vl53l0x-arduino) *Note that I have been using cheap, knockoff ToF sensors sourced from Amazon.com but this library has worked fine for me.*
 - Wire - Should be part of the Arduino core libraries.

### Optional Devtools:
  I do not like the Arduino IDE so I use arduino-cli to build and upload software to the tank.
  I have been using Ubuntu Linux to develop this software. If you use another OS or toolchain to
  develop software for LTRT, please feel invited to contribute to this documentation. I use the following in my dev toolchain:
 - Arduino-Makefile (https://github.com/sudar/Arduino-Makefile) I hate the Arduino IDE. I much prefer using vim to edit and Make to build/upload to the board.
In order to get this to work, I had to install the below arduino-cli and setup a configuration file for it (https://arduino.github.io/arduino-cli/1.1/configuration/) and install the arduino:avr platform]
 - arduino-cli (https://github.com/arduino/arduino-cli) Needed for above Arudino-Makefile

### Physical Requirements:
  You will, of course, need a Laser-Tag Robot Tank. You will also need a computer, a USB cable,  and an FTDI board in order to communicate with the Arduino chip on the LTRT circuit board.
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
ln -sf ./sketches/motor_strain_test.ino laser_tag_robot_tank.ino
```

## Code Documentation

I use Doxygen to automatically generate documentation. You can use a web browser to view the documentation
contained in this repo at [doxygen/html/index.html](doxygen/html/index.html). You will be most interested
in the Tank class, as the entrypoint for your code. I am currently working on adding better comments to
the library code.

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
I am an experienced software developer but I find it difficult to wrap my mind around a real-time system.
The important thing to grasp is that your loop() function will execute many times per second. So, if you
want to turn right 90 degrees and then left 45 degrees, you CANNOT do something like:
```
void loop() {
  tank.turn_right(90);
  tank.turn_left(45);
}
```

Because this will, in effect, continously negate the prior call by immediately trying to turn in the other
direction.

Instead, you must use some combination of timers, state variables, and callbacks in order to keep track of
what is happening and make the robot do what you want.

#### Coding Without Delay

The delay() function pauses all execution on the Arduino. You should NEVER use this function in the main code
of your robot. It is okay to use it the setup() function. Using delay() will cause the rest of the robot to
not work correctly, as it relies on the tank.loop() function being called continuously.

Instead you must use variables to keep track of time and state, such that you can allow some specified amount
of time to pass between actions. You can do a web-search for "arduino coding without delay" for a lot more
information on this subject.

### Your Own Bot

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

Look at the empty_bot.ino sketch if you want a good starting point.

### Limited Space

The ATmega328p boasts 32,256 bytes of flash memory and 2,048 bytes of SRAM. This memory can be taken
up fairly quickly so you'll have to program conservatively. As of this writing, an empty sketch that
includes all the libraries but doesn't in itself do anything at all takes up 8606/32256 byes of program
space and 634/2048 bytes SRAM. This leaves 23,650 and 1,414 bytes, respectively. That is all the
space you have left to make your robot the most intelligent and deadliest of all. Expect these numbers to
change slightly as I flesh-out the Tank library. But it is nearly complete so I don't expect to consume
much more space.

One simple trick, if you're running out of the very-limited SRAM, is to use the F() macro with your
print statements.

Instead of:
```
Serial.println("Robot initialized.");
```
Do this:
```
Serial.println("Robot initialized.");
```

The F() macro causes strings to be stored in the flash memory space, instead of SRAM.

### Common Pitfalls

#### Variable Types

You have to pay attention to variable types. I had a confusing bug the cause of which I was dividing
an integer with another integer and expecting a float in return. In the end I needed to do something
like:
```
return (float)integer_1 / integer_2;
```

### Upload Issues

Sometimes, when attempting to upload a new sketch, you might get an error like:

```
avrdude: stk500_recv(): programmer is not responding
avrdude: stk500_getsync() attempt 1 of 10: not in sync: resp=0x00
...
```

This happens even though the FTDI board is plugged into the programming header of LTRT. I'm not sure
exactly the cause, just like the Arduino brain is firing away and can't be stopped even though the
reset pin is being pulled to a low state.

Anyway, this can usually be fixed by unplugging and plugging back in the FTDI cable or by cycling the
power of the LTRT (unplugging the FTDI cable and turning the batteries off).

### Debugging

Sometimes the robot does not do what you want it to do and it is difficult to determine why. One way
to find the problem is to add debugging statements throughout the code. Some typical debug statement
for me will look like:

```
Serial.print("Value of current_degrees: ");
Serial.println(current_degrees);
```

Note that in the above Serial statements I am not using the F() macro. This means that my print strings
are taking up global variable space, which is fairly limited. However, I am adding these debug statements
just to track down some coding error I made and I intend to remove them once I find the problem.

Of course, printing these debug statements for every loop iteration will make for a gigantic log and might
even freeze communication through the FTDI. So will need to limit these print statements to every so often
(see section "Coding Without Delay", above).

Since I am using Linux, my debugging commands look something like:

```
> make upload
> screen -L /dev/ttyUSB0 115200
```

The above commands compile and upload a new version of the tank code. The screen command attaches my terminal
to the debugging output from the LTRT. The screen command with the -L flag will write-out a log file (named
something like screenlog.0). /dev/ttyUSB0 is the port (this might change based upon what USB devices are active
when you connect the LTRT). 115200 is the bits-per-second and this matches
a hardcoded value in my tank sketch.
