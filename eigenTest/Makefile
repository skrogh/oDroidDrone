CXX = g++
CFLAGS = -I

DEPS = odometry.hpp
OBJ = test.o odometry.o

%.o: %.x $(DEPS)
	$(CXX) -c -o $@ $< $(CFLAGS)

test: $(OBJ)
	$(CXX) -o $@ $^ $(CFLAGSs)