CXX = g++ -O0 -g
CFLAGS = $SHELL(pkg-config --cflags opencv)
LIBS = $SHELL(pkg-config --libs opencv)


DEPS = odometry.hpp
OBJ = test.o odometry.o



all: test

%.o: %.x $(DEPS)
	$(CXX) $(CFLAGS) $(LIBS) -c -o $@ $<

test: $(OBJ)
	$(CXX) $(CFLAGS) $(LIBS) -o $@ $^

clean: 
	$(RM) test *.o
