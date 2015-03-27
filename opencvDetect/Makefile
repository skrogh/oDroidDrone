CXX = g++ -O0 -g
CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`


DEPS = 
OBJ = test.o



all: test

%.o: %.x $(DEPS)
	$(CXX) $(CFLAGS) $(LIBS) -c -o $@ $<

test: $(OBJ)
	$(CXX) $(CFLAGS) $(LIBS) -o $@ $^

clean: 
	$(RM) test *.o
