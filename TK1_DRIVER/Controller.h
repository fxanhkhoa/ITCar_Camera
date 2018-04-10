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

#define ratio 0.5
#define multiplier 4096

class Controller
{
public:
  PCA9685 *pca9685;
  int err;

  Controller();
  ~Controller() ;
  void Initialize();
  void Speed(int left, int right);
};

#endif
