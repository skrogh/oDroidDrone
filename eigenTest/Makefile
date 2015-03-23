CXX = g++
CFLAGS = -O3

DEPS = odometry.hpp
OBJ = test.o odometry.o

%.o: %.x $(DEPS)
	$(CXX) -O3 -c -o $@ $<

test: $(OBJ)
	$(CXX) $(CFLAGS) -o $@ $^