CXX = g++ -O0 -g
CFLAGS = $(shell pkg-config --cflags opencv )
LIBS = $(shell pkg-config --libs opencv )


DEPS = feature.hpp odometry.hpp
OBJ = feature.o odometry.o



all: test

%.o: %.x $(DEPS)
	$(CXX) $(CFLAGS) -c -o $@ $< $(LIBS)

test: test.o $(OBJ)
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

opencvTest: opencvTest.o $(OBJ)
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

egienTest: egienTest.o $(OBJ)
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

clean: 
	$(RM) test egienTest opencvTest *.o
