FQBN?=arduino:avr:leonardo
PORT?=/dev/ttyACM0

all:
	arduino-cli compile --build-property "compiler.cpp.extra_flags=-I ../../include -I .. -include FreeRTOSConfig.h" --build-property "compiler.c.extra_flags=-include FreeRTOSConfig.h" --fqbn $(FQBN)

upload:
	arduino-cli upload --fqbn $(FQBN) -p $(PORT)

readlog: readlog.cpp
	${CXX} ${CXXFLAGS} -g -I ../../include -o $@ $^

install-deps:
	arduino-cli core install arduino:avr
	arduino-cli lib install FreeRTOS
