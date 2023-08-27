CC=g++
CFLAGS= -g -Wall
all: main
	./main

%.o: %.cpp 
	$(CC) $(CFLAGS) -c $@ $^

main: main.cpp graph.o
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -rf *.o main
