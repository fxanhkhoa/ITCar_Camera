#include "opencv2/imgproc/imgproc.hpp"
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

#define M_PI 3.14159265358979323846

// int min_mag_thresh = 0;
// int max_mag_thresh = 255;
// double min_thresh_angle = 0;
// double max_thresh_angle = M_PI / 2;

int ddepth = CV_16S;
int delta = 0;
int scale = 1;

Mat adp_thresh_grayscale(Mat gray, int threshold_val);

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

int main(int argc, char **argv) {
  Mat frame, output;
  VideoCapture cap("videos/clip1_FPT.mp4");
  if (!cap.isOpened())
    return -1;
  while (1) {
    cap >> frame;
    resize(frame, frame, Size(800, 600));
    GaussianBlur(frame, frame, Size(3, 3), 0, 0, BORDER_DEFAULT);
    cvtColor(frame, frame, CV_BGR2GRAY, 1);

    output = Mat::zeros(frame.size(), CV_8U);
    output = thresholding(frame, 200, 255, 0, 25, 200, 255, 0.7, 1.3, 113, 255,
                          234, 255, 15, 250);

    // output = abs_sobel_thresh(frame, 1, 3, 150,255);
    // output = dir_threshold(frame, 3, 0, M_PI/2);
    //output = mag_thresh(frame, 3, 200, 255);
    // imshow("a", output);
    /*for (int i = 0; i < output.rows; i++)
      for (int j = 0; j < output.cols; j++){
        cout <<
      }*/
    // cout << output << endl;
    // output = frame;

    int offset = 30;
    Mat mask = Mat::zeros(frame.size(), CV_8U);
    Point pts[4] = {
      Point(0 + offset, frame.rows),
      Point(frame.cols / 2.9, frame.rows / 1.95),
      Point(frame.cols / 1.5, frame.rows / 1.95),
      Point(frame.cols, frame.rows),
    };
    fillConvexPoly(mask, pts, 4, Scalar(255));
    bitwise_and(mask, output, frame);
    imshow("frame", frame);
    if (waitKey(30) == 27)
      break;
  }
  return 0;
}

Mat adp_thresh_grayscale(Mat gray, int threshold_val){
  Mat img, thresh;
  equalizeHist(gray, img);
  threshold(img, thresh, threshold_val, 255, THRESH_BINARY);

  return thresh;
}

Mat mag_thresh(Mat image, int sobel_kernel, int min_mag_thresh,
               int max_mag_thresh) {
  Mat sobelx, sobely, binary_output;

  // Take both Sobel x and y gradients
  Sobel(image, sobelx, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
  convertScaleAbs(sobelx, sobelx);
  Sobel(image, sobely, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
  convertScaleAbs(sobely, sobely);

  // Calculate the gradient magnitude
  float scale_factor = 0;
  Mat gradmag = Mat::zeros(image.size(), CV_8U);
  // unsigned char *input = (unsigned char*)(gradmag.data);
  // unsigned char *in_sobelx = (unsigned char*)(sobelx.data);
  // unsigned char *in_sobely = (unsigned char*)(sobely.data);
  // unsigned char *out = (unsigned char*)(binary_output.data);

  for (int i = 0; i < gradmag.rows; i++)
    for (int j = 0; j < gradmag.cols; j++) {
      // cout << sobelx.at<int>(j,i) << " ";
      // int x = in_sobelx[sobelx.step * j + i];
      // int y = in_sobely[sobely.step * j + i];
      uchar x = sobelx.at<uchar>(i, j);
      uchar y = sobely.at<uchar>(i, j);
      uchar k = uchar(sqrt(x * x + y * y));
      // cout << input[gradmag.step * j + i] << endl; // ko dc de sau ham ben
      // duoi
      // cout << k << " ";
      gradmag.at<uchar>(i, j) = (uchar)k;
      // cout << gradmag.at<uchar>(j,i) << endl;
      if (scale_factor < k)
        scale_factor = k;
      // cout << i << " "<< j<<endl;
    }
  scale_factor /= 255;
  //imshow("a", gradmag);
  // Create a binary image of ones where threshold is met, zeros otherwise
  binary_output = Mat::zeros(gradmag.size(), CV_8U);
  for (int i = 0; i < gradmag.rows; i++)
    for (int j = 0; j < gradmag.cols; j++) {
      // gradmag.at<uchar>(j,i) /= scale_factor;
      // cout << gradmag.at<uchar>(i,j) << " ";
      if ((gradmag.at<uchar>(i, j) >= min_mag_thresh) &&
          (gradmag.at<uchar>(i, j) <= max_mag_thresh))
        binary_output.at<uchar>(i, j) = 255;
      else
        binary_output.at<uchar>(i, j) = 0;
    }
  // cout << binary_output;
  return binary_output;
}

Mat dir_threshold(Mat image, int sobel_kernel, double min_thresh_angle,
                  double max_thresh_angle) {
  // Calculate the x and y gradients
  Mat sobelx, sobely, binary_output;
  Sobel(image, sobelx, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
  convertScaleAbs(sobelx, sobelx);
  Sobel(image, sobely, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
  convertScaleAbs(sobely, sobely);

  // Take the absolute value of the gradient direction,
  // apply a threshold, and create a binary image result
  Mat absgraddir = Mat::zeros(image.size(), CV_8U);
  binary_output = Mat::zeros(image.size(), CV_8U);
  for (int i = 0; i < absgraddir.rows; i++)
    for (int j = 0; j < absgraddir.cols; j++) {
      uchar x = sobelx.at<uchar>(i, j);
      uchar y = sobely.at<uchar>(i, j);
      double k = atan(double(abs(y)) / double(abs(x)));
      // cout << k <<" ";
      if ((k > min_thresh_angle) && (k < max_thresh_angle)) {
        binary_output.at<uchar>(i, j) = 255;
      }
    }
  return binary_output;
}

Mat abs_sobel_thresh(Mat image, int orient, int sobel_kernel,
                     int min_mag_thresh, int max_mag_thresh) {
  Mat abs_sobel = Mat::zeros(image.size(), CV_8U);
  uchar max = 0;
  if (orient == 1) {
    Mat sobelx;
    Sobel(image, sobelx, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(sobelx, sobelx);
    for (int i = 0; i < abs_sobel.rows; i++)
      for (int j = 0; j < abs_sobel.cols; j++) {
        abs_sobel.at<uchar>(i, j) = abs(sobelx.at<uchar>(i, j));
        if (max < abs(sobelx.at<uchar>(i, j)))
          max = abs(sobelx.at<uchar>(i, j));
      }
  } else if (orient == 2) {
    Mat sobely;
    Sobel(image, sobely, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(sobely, sobely);
    for (int i = 0; i < abs_sobel.rows; i++)
      for (int j = 0; j < abs_sobel.cols; j++) {
        abs_sobel.at<uchar>(i, j) = abs(sobely.at<uchar>(i, j));
        if (max < abs(sobely.at<uchar>(i, j)))
          max = abs(sobely.at<uchar>(i, j));
      }
  }

  Mat binary_output = Mat::zeros(abs_sobel.size(), CV_8U);
  // Mat scaled_sobel = Mat::zeros(abs_sobel.size(), CV_8U);
  for (int i = 0; i < abs_sobel.rows; i++)
    for (int j = 0; j < abs_sobel.cols; j++) {
      uchar k = 255 * abs_sobel.at<uchar>(i, j) / max;
      if ((k > min_mag_thresh) && (k < max_mag_thresh)) {
        if (orient == 1)
          binary_output.at<uchar>(i, j) = 255;
      }
      else {
        if (orient == 2)
          binary_output.at<uchar>(i, j) = 255;
      }
    }
  return binary_output;
}

Mat thresholding(Mat image, int grad_thx_min , int grad_thx_max ,
                 int grad_thy_min , int grad_thy_max ,
                 int mag_th_min , int mag_th_max ,
                 double dir_th_min , double dir_th_max ,
                 int s_threshold_min , int s_threshold_max ,
                 int v_threshold_min , int v_threshold_max ,
                 int k_size , int adp_thr ) {
  // threshold grad
  // Choose a larger odd number to smooth gradient measurements
  int ksize = k_size;
  Mat gradx = Mat::zeros(image.size(), CV_8U);
  gradx = abs_sobel_thresh(image, 1, ksize, grad_thx_min, grad_thx_max);
  imshow("gradx",gradx);
  Mat grady = Mat::zeros(image.size(), CV_8U);
  grady = abs_sobel_thresh(image, 2, ksize, grad_thy_min, grad_thy_max);
  imshow("grady",grady);
  Mat mag_binary = Mat::zeros(image.size(), CV_8U);
  mag_binary = mag_thresh(image, ksize, mag_th_min, mag_th_max);
  //imshow("c",mag_binary);
  Mat dir_binary = Mat::zeros(image.size(), CV_8U);
  dir_binary = dir_threshold(image, ksize, dir_th_min, dir_th_max);
  //imshow("d",dir_binary);
  Mat combined = Mat::zeros(gradx.size(), CV_8U);

  for (int i = 0; i < combined.rows; i++)
    for (int j = 0; j < combined.cols; j++) {

      if (((gradx.at<uchar>(i, j) == 255) && (grady.at<uchar>(i, j) == 255)) ||
          ((mag_binary.at<uchar>(i, j) == 255) &&
          (dir_binary.at<uchar>(i, j) == 255))) {
        combined.at<uchar>(i, j) = 255;
      }
    }
  //imshow("combined", combined);
  return combined;
  Mat adp;
  adp = adp_thresh_grayscale(image, adp_thr);
  for (int i = 0; i < adp.rows; i++)
    for (int j = 0; j < adp.cols; j++){
      adp.at<uchar>(i,j) /= 255;
    }

  Mat color_binary = Mat::zeros(gradx.size(), CV_8U);
  for (int i = 0; i < color_binary.rows; i++)
    for (int j = 0; j < color_binary.cols; j++){
      if ((combined.at<uchar>(i,j) == 255) && (adp.at<uchar>(i,j) == 255))
        color_binary.at<uchar>(i,j) = 255;
    }
  //return color_binary;
}
