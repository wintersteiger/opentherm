all: readlog

readlog: readlog.cpp
	${CXX} ${CXXFLAGS} -g -I ../../../include -o $@ $^ -lpqxx -lpq

clean:
	rm -rf *.o *.d readlog