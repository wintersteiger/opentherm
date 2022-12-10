all: sim

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

sim: sim.o setspeed.o
	$(CXX) -o $@ $^
