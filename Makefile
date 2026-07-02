BOARD = arduino:avr:uno
PATH = $(PWD)
# won't work with FTDI
#$(eval PORT=$(shell sh -c "arduino-cli board list | grep arduino:avr" | awk '{print $$1}'))
PORT = "/dev/ttyUSB0"

compile:
	/usr/local/bin/arduino-cli compile --library $(PWD)/src -b $(BOARD) $(PATH)

upload: compile
	/usr/local/bin/arduino-cli upload -b $(BOARD) -p $(PORT) $(PATH)

docs: FORCE
	/usr/bin/doxygen Doxyfile

FORCE: ;
