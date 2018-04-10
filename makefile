CC=g++
CFLAGS=-g -Wall
OPENCV = `pkg-config opencv --cflags --libs`
LIBS=$(OPENCV)

mainmake: ItcarVanishingpoint.cpp
	$(CC) ItcarVanishingpoint.cpp -o output $(LIBS)

clean:
	rm -rf *o output
