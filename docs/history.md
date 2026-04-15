# History

Just some ramblings on the history of this project.

## Inspiration

The concept for this project was formed sometime around 2016. I had been playing around with Arduino and electronics in general for a while. I was
interested in creating some kind of autonomous robot. I had also been interested in the Pixart camera that came with the Wiimote. It had made
something of a big splash in the hobbyist electronics world as it was a camera (albeit, a low-resolution, strictly IR camera) that was pretty
cheap and had a decent refresh rate. I don't remember when the idea fully formed but I purchased IR battle tanks off of Amazon and it occurred to
me that I could make them autonomous.

I have long been interested in tank battles. I mean cartoonish, abstracted battles. Not actual tank battles. When I was a kid, my brother and I played
a game called Combat on the Atari 2600. You could battle tanks, prop planes, or jet fighters in various modes. But our favorite was the tanks. In particular
I remember really enjoying the mode where your tank was only visible for a moment, when you took a shot. We wore-out multiple Atari joysticks battling
each other. In high-school, the first ambitious program I made was a clone of Combat using QBasic. Later, in college, I wrote another clone using C++
and SDL. I started on some system to program bots using a scripting language but I never got that part working very well. It was also networked but
I wrote my own networking code and that part didn't work very well, either. Anyway, it seemed to make sense to attempt to make some real-life version of the game.

## Development

Sometime in 2016, I bought a set of RC tanks off of Amazon. They "battled" each other using IR signals, which is what all "laser" tag systems use.
An IR LED in the tank barrel "shot" at a dome situated on the top of the turrent of the opposite tank, which registered the hit. Four LEDs functioned
as a hit counter and there were various other effects like shooting noises and a recoil effect wherein the tank backed-up a little every time you
made a shot.

I disassembled the tanks and experimented with them by adding my own components. These tanks had a large ring gear that spun the turret around. Each
ring gear had a "stopper" section where, instead of gear teeth, the gear was just a solid piece. They had to do this because they could not allow the
turret to spin a full 360 degrees, lest the wires going to the turret get twisted. The turret motor had a clutch system where the gears would slip, instead
 of stalling the motor. It made a terrible racket, regardless. My very first modification was to use a Dremel to cut-out the blank portion of the ring
 gear into something resembling teeth. I added a slip-ring for the wiring. I placed a Pixart camera on top of the turret and it communicated with an
 Arduino development board in the main body of the tank. So I had an initial version that could acquire and track an IR target back and forth. I was
 also experimenting with ultrasonic distance sensors, as the tank would need some way to navigate its environment.

The hand-cut ring gear sorta worked but it was tricky to cut out the teeth correctly, what with the plastic melting and all. I ended up modeling the
gear using FreeCAD and I sent-away to Shapeways to print the gear. This was a much better gear, albeit more expensive and with a somewhat lengthy turn-around time.

Then life intervened, I moved to Arizona for a while, all my electronics stuff sat in a box in a storage unit.

Fast forward to 2020, I was living in Portland and I had space to play with robots. Also, the pandemic was under way so I found myself house-bound
and with extra money and time. I wanted to revive the tank project but I had to decide what to do about the physical construction of the tank. I had
purchased a few more IR battle tanks from Amazon. Some of them were easy to modify but others didn't really have enough internal space to add electronics
to. They fluctuated in price but they were all fairly expensive for my purpose considering that I only really used the body, motors, and running gear.
All the electrics went into the trash. I figured I would only need a few custom parts printed but buying those on-demand was fairly expensive and there
was a turnaround time when testing prototypes. So, anyway, the idea of buying a 3D printer had always appealed to me, though it was daunting. I was never
really interested in the finer details of 3D printing. I'm still not very interested in it.

So stuck at home during the pandemic, I was left to choose between buying an entire lot of IR-battling robots and purchasing a 3D printer. If I purchased a lot
of 50 or 100 robots, the cost per-tank would be reasonable and I would have a consistent design with which to experiment with prototypes. In the end, I would
have plenty of copies to hand out to friends. But it was wasteful and actually modifying a plastic tank to fit my electronics package was difficult and produced
inconsistent results. In the end, I decided to buy an Ender 5 and learn about CAD software and 3D printing and make my tank from scratch.
This turned out to be the right choice but was of course a whole entire learning experience.

The disassembly of these injection-molded plastic tanks proved to be instructional. Their DNA lives on in the current version of my tank. The turret ring gear
I use is an almost-exact copy of the ring gear I pulled from one of those tanks.

So of course 3D modeling is its own disciple and I never became much more than a hack. I'd consider myself a hack with electronics, also. Software is my actual
profession and the only thing in this project that I felt competent at. 3D printing gears in particular can be a bit tricky, especially as they get smaller. The
turret gears in this tank work well enough but there is quite a bit of lash and they are noisy. For quality, nothing can beat the Shapeways gear I purchased years
ago. They use an SLS process that produces great results.

Back to the development story. Around 2022, I was moving out of the house I lived in in Portland and so I once again had to pack up the workbench. During the pandemic
I had made some great progress. I had a mostly-complete design for the 3D-printed tank, I had designed circuit boards for the main board and the turret, as well as boards for
the LEDs and bump sensors. The one issue that bedeviled me was that motor activity would shut-down the logic controller and trigger a reset. I had long had issues
with motor drivers interferring with the IC. Everything would be working fine but running the motor at full-speed and then reversing direction would sometimes trigger
a microcontroller reset. For a while I had attempted to run everything off of 4 1.5A AA NiCad batteries. I liked this setup as it was simple and didn't require to
switch out a bunch of batteries. Because of my reset issues, I reluctantly moved to running the microcontroller of a 9V battery. But still the reset issues persisted.
  I was frustrated and out of development time, for the time-being, anyway.

I moved to a new house and life kinda took over and I didn't have time for robots. Eventually, around 2025, things settled down a bit and I had extra time for hobbies.
I did kinda deliberate whether I wanted to pick back up with the LTRT project again. It had last ended in frustration and I spent a lot of time on it without much to show.
 On the other hand, I had spent a lot of time on it without much to show. So I pulled out the electronics. The old, dust-covered Ender came out of the shop. With just
 a quick wipe-down and bed-level, it was back up and working. I took a look at my main logic board again, with fresh eyes. Of course, I could have easily used off-the-shelf
 motor drivers to do what I wanted. But this did not appeal to me. Part of it is I guess a stubborn streak of DIY-ness. But the other thing was that I had to run three
 motors so I would need two full-bridge drivers and these don't tend to be cheap. Not the Pololu ones, anyway. My aim had always been to design something with a low
 per-unit cost. Well, that wasn't possible anymore but with a reasonable per-unit cost, anyway. I could have bought some cheap, knockoff drivers off of Amazon and probably
 saved myself a bunch of headache but I guess that hasn't really been the ethos of this project.

So, anyway, I was doing research on the Internet and trying to debug my circuit design and I ended-up figuring out that my board was basically one big ground loop. I had
put ground planes on both sides of the board plus I had ground traces everywhere. The reason for this is because, at some point, I had been convinced that my reset issues
had been because my motor drivers were overheating, and I needed more heat-sink capability. But all this ground path did was create a bunch of electrical noise whenever
a motor stopped turning and all that energy was fed back into the board. I was using chips such as the L293D which claim to have internal diodes to combat this but I guess
whatever noise I was producing was too much. Anyway, the fix seemed to be to reduce the ground plane to a smaller size, on a single side of the board, as well as to elimate
traces that would create a "loop" effect. I can't claim that this is the definitive fix, as I can't be sure that I won't run into the same problems down the road, as I do
more vigorous testing of the robot. Like I said, I'm not really an electronics expert.

In 2025 I had a mostly-complete prototype. It drove around and, when it bumped into things, it reversed direction. All of the physical functions that I wanted to work did
and I was able to focus on my favorite part, the part I am best at, the software. So I wrote a bunch of code to kinda abstract some of the gritty detail of tank operation 
away from the user. I mean things like keeping track of wheel position and turret position and stuff like that. The code forced some delay between reversing the motors as,
apparently, placing opposite current on a motor while it is still spinning is bad for the motor.

So I showed the prototype around to some friends and the feedback was positive but when I asked about additional capabilities everyone agreed that better batteries would
be nice and some kind of obstacle sensing would be nice.

The motors had always been rather sluggish running off of a 4.5V battery pack. It was going to be difficult to fit 6 1.5V batteries without making the tank chassis a lot 
bigger. I never really worried about battery life so the motors didn't usually run very long while I was doing development work. Anyway, I had always sorta resisted Lithium
batteries because, frankly, I was scared of them. I didn't want the expense of a Li-ion charge controller on the tank. Also, the lazy part of me didn't want to do the chassis
redesign that would be necessary to accomodate a different battery pack.

In the end, I discovered that 18650 batteries hold a nice charge. A two-pack gives me 7.2 volts to the motors, a nice upgrade from the ~4.5 volts I was seeing with NiCad batteries.
Two 18650 batteries and a dedicated charger for them was not too expensive. The prior design iteration required to remove an axle to access the underside battery pack. So I did a
slight redesign in order to elimate the axles and accomodate the 18650 pack.

The distance sensing was the other angle I was working on. Early on, I had been experimenting with ultrasonic sensors like the HC-SR04. It had occurred to me that having two
robots meant that ultrasonic signals might interfere with each other. As I messed around with the sensors, I found them to be somewhat finicky and I realized that I did not in fact
have any method of dealing with the interference issue. So I just eliminated them and stuck to bump sensors. But I never did really find this elegant, the notion that my tank
robot was forced to bump around a room, blind as a bat, except for its camera which only sensed infrared light.

When I looked at the options available, I saw that knockoff boards for the VL53L0X were quite cheap. That option wasn't available to me when I started this project. These sensors
use time-of-flight calculations, using infrared light reflecting off of surfaces. That you can buy a module for $5 with circuitry sophisticated enough to get sub-centimeter accurracy
measuring the speed of light is just one of the many wonders of our modern world. Anyway, best of all, the VL53L0X boards use I2C to communicate with the microcontroller. Since I was
already using I2C to talk to the camera, on a different address, this meant that I would lose no GPIO pins in order to add distance-sensing. When you start your first project with 
Arduino it seems like there are so many pins you'd never know what to do with them all. But they quickly fill up and you always end up needing more pins than you have.
