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
    pca9685->setPWMFrequency(60);
    sleep(1);
    pca9685->setPWM(STEERING_CHANNEL, 0, 420);
    Speed(0,0);
}

Controller::~Controller() {}

void Controller::Speed(int left, int right) {
  int real_left = ((left * multiplier) / 100) * ratio;
  int real_right = ((right * multiplier) / 100) * ratio;

  if (real_left < 0)
  {
    pca9685->setPWM(MOTOR_LEFT_IN1, 0, 0);
    pca9685->setPWM(MOTOR_LEFT_IN2, 0, -real_left);
  }
  else
  {
    pca9685->setPWM(MOTOR_LEFT_IN1, 0, real_left);
    pca9685->setPWM(MOTOR_LEFT_IN2, 0, 0);
  }

  if (real_right < 0)
  {
    pca9685->setPWM(MOTOR_RIGHT_IN1, 0, 0);
    pca9685->setPWM(MOTOR_RIGHT_IN2, 0, -real_right);
  }
  else
  {
    pca9685->setPWM(MOTOR_RIGHT_IN1, 0, real_right);
    pca9685->setPWM(MOTOR_RIGHT_IN2, 0, 0);
  }
}

void Controller::Handle(int angle)
{
  int percentage = (angle * 100) / 90;
  //percentage *= angle_ratio;
  int angle_pwm;
  if (percentage < 0)
  {
    angle_pwm = ((-percentage) * ((servoMax - servoMin) / 2))/100;
	cout <<angle_pwm<<endl;
    angle_pwm = ((servoMax - servoMin) / 2 + servoMin) - (angle_pwm * angle_ratio);
	cout<<angle_pwm;
    pca9685->setPWM(STEERING_CHANNEL, 0, angle_pwm);
  }
  else
  {
    angle_pwm = ((percentage) * ((servoMax - servoMin) / 2))/100;
    angle_pwm = ((servoMax - servoMin) / 2 + servoMin) + (angle_pwm * angle_ratio);
	cout<<angle_pwm;
    pca9685->setPWM(STEERING_CHANNEL, 0, angle_pwm);
  }
}

void Controller::Initialize() {}
