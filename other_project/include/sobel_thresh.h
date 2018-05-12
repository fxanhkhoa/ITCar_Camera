#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "opencv2/imgproc/imgproc.hpp"
#include <cmath>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

using namespace std;
using namespace cv;

Mat adp_thresh_grayscale(Mat gray, int threshold_val);
Mat get_perspective(Mat frame, Mat src);
Mat mag_thresh(Mat image, int sobel_kernel, int min_mag_thresh,
               int max_mag_thresh);
Mat dir_threshold(Mat image, int sobel_kernel, double min_thresh_angle,
                  double max_thresh_angle);
Mat abs_sobel_thresh(Mat image, int orient, int sobel_kernel,
                     int min_mag_thresh, int max_mag_thresh);
Mat thresholding(Mat image, int grad_thx_min, int grad_thx_max,
                 int grad_thy_min, int grad_thy_max, int mag_th_min,
                 int mag_th_max, double dir_th_min, double dir_th_max,
                 int s_threshold_min, int s_threshold_max, int v_threshold_min,
                 int v_threshold_max, int k_size, int adp_thr);
double *polyfit(double leftx[], double lefty[], int count, int degree);
double *sliding_window(Mat frame);

#endif
