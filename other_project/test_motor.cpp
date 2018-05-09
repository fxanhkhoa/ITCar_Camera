#include "TK1_DRIVER/Controller.h"
#include "TK1_DRIVER/jetsonGPIO.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <string>
#include <unistd.h>

using namespace std;

int getkey();

int main()
{
	jetsonGPIONumber BTN1 = gpio160;
	jetsonGPIONumber BTN2 = gpio161;
	jetsonGPIONumber BTN3 = gpio163;
	jetsonGPIONumber BTN4 = gpio164;
	gpioExport(BTN1);
    	gpioExport(BTN2);
	gpioExport(BTN3);
	gpioExport(BTN4);
    	gpioSetDirection(BTN1,inputPin);
	gpioSetDirection(BTN2,inputPin);
	gpioSetDirection(BTN3,inputPin);
	gpioSetDirection(BTN4,inputPin);

	Controller *_Controller = new Controller();

	// Wait for the push button to be pressed
   	cout << "Please press the button! ESC key quits the program" << endl;

	unsigned int value = high;

	while(getkey() != 27) {
		gpioGetValue(BTN1, &value);
		if (value == low){
			_Controller->Speed(100,100);
			_Controller->Handle(90);
		}
		gpioGetValue(BTN2, &value);
		if (value == low){
			_Controller->Handle(-90);
			_Controller->Speed(0,0);
		}
	}
	
	cout << "End" << endl;
	gpioUnexport(BTN1);
    	gpioUnexport(BTN2);
	gpioUnexport(BTN3);
	gpioUnexport(BTN4);
	_Controller->Speed(0,0);
}


int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

