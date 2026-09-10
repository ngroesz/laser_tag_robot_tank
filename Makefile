BOARD = arduino:avr:uno
PATH = $(PWD)
$(eval PORT=$(shell sh -c "arduino-cli board list | grep -E 'arduino:avr|Serial Port \(USB\)'" | awk '{print $$1}'))

compile:
	/usr/local/bin/arduino-cli compile --library $(PWD)/src -b $(BOARD) $(PATH)

clean:
	/usr/local/bin/arduino-cli compile --clean --library $(PWD)/src -b $(BOARD) $(PATH)

upload: compile
	/usr/local/bin/arduino-cli upload -b $(BOARD) -p $(PORT) $(PATH)

docs: FORCE
	/usr/bin/doxygen Doxyfile

FORCE: ;
