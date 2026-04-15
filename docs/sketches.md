# Sketches

This repository comes with several sketches, located under sketches/, that demonstrate what the
Laser-Tag Robot Tank is capable of. I attempted to add useful comments to these sketches, such
that someone trying to learn how to code their own bot can obtain information and inspiration.

## hardware_test.ino

If you have a newly-assembled tank, this it the first sketch that you should upload to the robot.
You will need an IR remote in order to use this sketch. This sketch is meant to demonstrate the
various capabilities of the robot and to ensure that all functions are in working order. You can
follow the guide below in order to command the robot to do various things.

TODO: add guide

## dumb_bot.ino

A very dumb bot in that it has no intelligence whatsover. It merely initalizes the tank and
registers callback functions. A good starting template if you want to write your own bot.

## wander_bot.ino

Wander bot is made to wander around a space, avoiding obstacles. It uses the distance sensor
to measure the distance to the nearest obstacle. It keeps track of the nearest obstacles that
exist in a 360-degree view. More details in the code.
