BOARD = arduino:avr:uno
PATH = /home/nick/laser_tag_robot_tank/dev_sketches/simple_target
PORT = /dev/ttyACM3

.PHONY:
compile:
	/usr/local/bin/arduino-cli compile -b $(BOARD) $(PATH)

upload:
	/usr/local/bin/arduino-cli upload -b $(BOARD) -p $(PORT) $(PATH)

