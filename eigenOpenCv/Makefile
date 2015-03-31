CXX = g++ -O0 -g
CPPFLAGS = $(shell pkg-config --CPPFLAGS opencv )
LIBS = $(shell pkg-config --libs opencv )


DEPS = feature.hpp odometry.hpp
OBJ = feature.o odometry.o



all: test

%.o: %.x $(DEPS)
	$(CXX) $(CPPFLAGS) -c -o $@ $< $(LIBS)

test: test.o $(OBJ)
	$(CXX) $(CPPFLAGS) -o $@ $^ $(LIBS)

opencvTest: opencvTest.o $(OBJ)
	$(CXX) $(CPPFLAGS) -o $@ $^ $(LIBS)

egienTest: egienTest.o $(OBJ)
	$(CXX) $(CPPFLAGS) -o $@ $^ $(LIBS)

clean: 
	$(RM) test egienTest opencvTest *.o
