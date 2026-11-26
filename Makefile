BOARD       = arduino:avr:uno
SKETCH      = $(CURDIR)
ARDUINO_CLI = /usr/local/bin/arduino-cli

.PHONY: compile clean format upload docs

compile:
	$(ARDUINO_CLI) compile --library "$(CURDIR)/src" -b $(BOARD) "$(SKETCH)"

clean:
	$(ARDUINO_CLI) compile --clean --library "$(CURDIR)/src" -b $(BOARD) "$(SKETCH)"

format:
	/usr/bin/find . -type f \( -iname "*.h" -o -iname "*.cpp" -o -iname "*.ino" \) -print0 | /usr/bin/xargs -0 -r /usr/bin/clang-format -i

upload: compile
	@port="$(PORT)"; \
	if [ -z "$$port" ]; then \
		port=$$($(ARDUINO_CLI) board list | awk '/arduino:avr|Serial Port \(USB\)/ {print $$1; exit}'); \
	fi; \
	if [ -z "$$port" ]; then \
		echo "No Arduino port found. Run: make upload PORT=/dev/ttyXXX" >&2; \
		exit 1; \
	fi; \
	echo "Uploading to $$port"; \
	$(ARDUINO_CLI) upload -b $(BOARD) -p "$$port" "$(SKETCH)"

docs:
	/usr/bin/doxygen Doxyfile
