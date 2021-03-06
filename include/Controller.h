#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <cstddef>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "JHPWMPCA9685.h"
#include <iostream>

using namespace std;

//#define ratio	0.6
#define angle_ratio 0.6
#define multiplier 4096
#define STEERING_CHANNEL 0
#define MOTOR_LEFT_IN1  4
#define MOTOR_LEFT_IN2  5
#define MOTOR_RIGHT_IN1  6
#define MOTOR_RIGHT_IN2  7

using namespace std;

class Controller
{
public:
  PCA9685 *pca9685;
  int err;
  int servoMin = 120 ;
  int servoMax = 720 ;

  Controller();
  ~Controller() ;
  void Initialize();
  void Speed(int left, int right);
  void Handle(int angle);
};

#endif
