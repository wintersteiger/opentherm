all: readlog

readlog: readlog.cpp
	${CXX} ${CXXFLAGS} -g -I ../../include -o $@ $^

clean:
	rm -rf *.o *.d readlog