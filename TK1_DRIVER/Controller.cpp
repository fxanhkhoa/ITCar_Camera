#include "Controller.h"

Controller::Controller() {
  pca9685 = new PCA9685();
  err = pca9685->openPCA9685();
  if (err < 0){
        printf("Error: %d", pca9685->error);
    }
    printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
    pca9685->setAllPWM(0,0) ;
    pca9685->reset() ;
    pca9685->setPWMFrequency(50);
    sleep(1);
}

Controller::~Controller() {}

void Controller::Speed(int left, int right) {
  int real_left = ((left * multiplier) / 100) * ratio;
  int real_right = ((right * multiplier) / 100) * ratio;

}

void Controller::Initialize() {}
