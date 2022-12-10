FQBN?=arduino:avr:leonardo
PORT?=/dev/ttyACM0

all:
	arduino-cli compile --build-property "compiler.cpp.extra_flags=-I ../include -include FreeRTOSConfig.h" --build-property "compiler.c.extra_flags=-include FreeRTOSConfig.h" --fqbn $(FQBN)

upload:
	arduino-cli upload --fqbn $(FQBN) -p $(PORT)

install-deps:
	arduino-cli core install arduino:avr
	arduino-cli lib install FreeRTOS
