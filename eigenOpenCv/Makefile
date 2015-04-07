CXX = g++ -O3 -g
CPPFLAGS = $(shell pkg-config --cflags opencv )
LIBS = $(shell pkg-config --libs opencv ) -pthread


DEPS = feature.hpp odometry.hpp common.hpp imu.hpp quaternionAlias.hpp
OBJ = feature.o odometry.o common.o imu.o



all: test

%.o: %.x $(DEPS)
	$(CXX) $(CPPFLAGS) -c -o $@ $< $(LIBS)

test: test.o $(OBJ)
	$(CXX) $(CPPFLAGS) -o $@ $^ $(LIBS)

opencvTest: opencvTest.o $(OBJ)
	$(CXX) $(CPPFLAGS) -o $@ $^ $(LIBS)

eigenTest: eigenTest.o $(OBJ)
	$(CXX) $(CPPFLAGS) -o $@ $^ $(LIBS)

imuTest: imuTest.o $(OBJ)
	$(CXX) $(CPPFLAGS) -o $@ $^ $(LIBS)

matlabUnitTest: matlabUnitTest.o $(OBJ)
	$(CXX) $(CPPFLAGS) -o $@ $^ $(LIBS)

clean: 
	$(RM) test egienTest opencvTest imuTest *.o
