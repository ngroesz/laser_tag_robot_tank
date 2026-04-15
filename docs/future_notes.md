Just to jot-down some possible future developments.

Cost savings:
    * Using simpler gearboxes or more 3D-printed parts (see https://www.thingiverse.com/thing:2770957 for the example of Tank-Go!)
    * using a photoresistor and rotary encoders. This would work at least for the wheel encoders and would be much cheaper than the Hall-effect wheel encoder kits.

More better:
    * Using a single battery supply. This was in my original design and was abandoned because of all the issues I had with motor drivers. Now that my motor driver issues appear to be solved, I could look at this again. One consideration is that I could likely not use 4 AAs to drive my microcontroller at 5V. Four rechargeable AAs produce something like 4.8V at full charge. The linear regulator drops further voltage. I would either need to run all my ICs at 3.3V or boost the voltage before regulating it. I don't like either of those ideas. I could also add more batteries and I am looking into the possibility of mounting a holder for six AA batteries. This would give me 7.2 volts which would be great for both motors and logic. There is the remaining problem that motor use could disrupt the logic voltage supply and this is the reason most everyone warns you to not do it this way. But it's worth trying to see if it's feasible, probably with several capacitors between the batteries and the 5V linear regulator. I could also look into lithium batteries but I'm not super keen on that idea.

Far off or not likely to happen:
    * Distance sensors. My early experiments with ultrasonic distance sensors indicated that there would be issues with multiple robots and multiple sensors. Could also consider IR distance sensors. This would certainly increase cost over my bump sensors but it would be neat to "navigate" obstacles rather than simply bumping into them.
