CC=g++
CFLAGS=-g -Wall
OPENCV = `pkg-config opencv --cflags --libs`
LIBS=$(OPENCV)

mainmake: ItcarVanishingpoint.cpp TK1_DRIVER/Controller.cpp TK1_DRIVER/JHPWMPCA9685.cpp
	$(CC) ItcarVanishingpoint.cpp TK1_DRIVER/Controller.cpp TK1_DRIVER/JHPWMPCA9685.cpp -o output $(LIBS)
	@echo 'done'


clean:
	rm -rf *o output
