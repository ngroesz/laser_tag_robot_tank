# Quickstart

The quickest possible way to get your robot tank to do something.

1. Install the required software.
  - Check out the Requirements section in the [development](development.md) file.
  - If you don't want to mess with arduino-cli just yet, you can use the [offical Arduino software](https://www.arduino.cc/en/software/).
  - clone the [laser_tag_robot_tank git repo](https://github.com/ngroesz/laser_tag_robot_tank)

2. Within the laser_tag_robot_tank repo, make sure that laser_tag_robot_tank.ino is linked to the hardware_test.ino sketch.
`
ln -sf ./sketches/hardware_test.ino laser_tag_robot_tank.ino
`
3. Upload the sketch using the Arduino software or arduino-cli.
Using arduino-cli, from within the laser_tag_robot_tank directory:
`
make upload
`

4. Make sure batteries are installed in the tank and turn it on using the switch on the front of the battery holder.

5. Using your IR remote, run the tank through its paces.

The [hardware guide](hardware_guide.md) will tell you the commands that you can run that will test the various functions of the laser tag robot tank.

6. Go on and develop your own software.

Now that you have the basics, check out the [development guide](development.md) to learn more about writing code for the laser tag robot tank.
