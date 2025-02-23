BOARD = arduino:avr:uno
PATH = /home/nick/laser_tag_robot_tank/dev_sketches/simple_target
# my attempt to automatically detect port
$(eval PORT=$(shell sh -c "arduino-cli board list | grep arduino:avr" | awk '{print $$1}'))

.PHONY:
compile:
	/usr/local/bin/arduino-cli compile -b $(BOARD) $(PATH)

upload:
	/usr/local/bin/arduino-cli upload -b $(BOARD) -p $(PORT) $(PATH)

