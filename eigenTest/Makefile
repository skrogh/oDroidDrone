CXX = g++
CFLAGS = -I

DEPS =
OBJ = test.o

%.o: %.x $(DEPS)
	$(CXX) -c -o $@ $< $(CFLAGS)

test: $(OBJ)
	$(CXX) -o test $@ $^ $(CFLAGSs)