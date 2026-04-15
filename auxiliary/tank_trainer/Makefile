BOARD = arduino:avr:uno
PATH = $(PWD)
# won't work with FTDI
#$(eval PORT=$(shell sh -c "arduino-cli board list | grep arduino:avr" | awk '{print $$1}'))
PORT = "/dev/ttyUSB0"

compile:
	/usr/local/bin/arduino-cli compile -b $(BOARD) $(PATH)

upload:
	/usr/local/bin/arduino-cli upload -b $(BOARD) -p $(PORT) $(PATH)
