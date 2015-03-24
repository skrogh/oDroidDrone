CXX = g++ -O0 -g -J4
CFLAGS =

DEPS = odometry.hpp
OBJ = test.o odometry.o



all: test

%.o: %.x $(DEPS)
	$(CXX) -c -o $@ $<

test: $(OBJ)
	$(CXX) $(CFLAGS) -o $@ $^

clean: 
	$(RM) test *.o
