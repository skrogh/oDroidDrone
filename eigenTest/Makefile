CXX = g++
CFLAGS = -g

DEPS = odometry.hpp
OBJ = test.o odometry.o

%.o: %.x $(DEPS)
	$(CXX) $(CFLAGS) -c -o $@ $<

test: $(OBJ)
	$(CXX) $(CFLAGS) -o $@ $^