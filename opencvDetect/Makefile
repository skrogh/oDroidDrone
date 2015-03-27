CXX = g++ -O0 -g
CFLAGS = $(shell pkg-config --cflags opencv )
LIBS = $(shell pkg-config --libs opencv )


DEPS = 
OBJ = test.o



all: test

%.o: %.x $(DEPS)
	$(CXX) $(CFLAGS) -c -o $@ $< $(LIBS)

test: $(OBJ)
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

clean: 
	$(RM) test *.o
