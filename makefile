CC=g++
CFLAGS=-g -Wall
OPENCV = `pkg-config opencv --cflags --libs`
LIBS=$(OPENCV)

mainmake: ItcarVanishingpoint.cpp TK1_DRIVER/Controller.cpp TK1_DRIVER/JHPWMPCA9685.cpp
	$(CC) ItcarVanishingpoint.cpp TK1_DRIVER/Controller.cpp TK1_DRIVER/JHPWMPCA9685.cpp -o output $(LIBS)
	@echo 'done'


testmotor: test_motor.cpp TK1_DRIVER/Controller.cpp TK1_DRIVER/JHPWMPCA9685.cpp TK1_DRIVER/jetsonGPIO.c
	$(CC) test_motor.cpp TK1_DRIVER/Controller.cpp TK1_DRIVER/JHPWMPCA9685.cpp TK1_DRIVER/jetsonGPIO.c -I TK1_DRIVER -o testmotor
	@echo 'done'

testcam: main.cpp
	$(CC) main.cpp -o testcam $(LIBS)

clean:
	rm -rf *o output
