FQBN?=arduino:avr:leonardo
PORT?=/dev/ttyACM0

all:
	arduino-cli compile --fqbn $(FQBN)

upload:
	arduino-cli upload --fqbn $(FQBN) -p $(PORT)

install-deps:
	arduino-cli core install arduino:avr
	arduino-cli lib install FreeRTOS
