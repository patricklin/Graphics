CC = g++
RM = /bin/rm -f
CFLAGS = -g -I . -O3


all: main

main: as2.o lodepng.o
	$(CC) $(CFLAGS) as2.o lodepng.o -o raytracer

as2.o: as2.cpp 
	$(CC) $(CFLAGS) -c as2.cpp -o as2.o

lodepng.o: lodepng.cpp
	$(CC) $(CFLAGS) -c lodepng.cpp -o lodepng.o

clean: 
	rm -f *.o raytracer *.png