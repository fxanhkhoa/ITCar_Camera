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

#define M_PI  3.14159265358979323846

int min_mag_thresh = 0;
int max_mag_thresh = 255;
double min_thresh_angle = 0;
double max_thresh_angle = M_PI / 2;


int ddepth = CV_16S;
int delta = 0;
int scale = 1;

Mat mag_thresh(Mat image, int sobel_kernel);

int main(int argc, char** argv ){
  Mat frame, output;
  VideoCapture cap("videos/clip1_FPT.mp4");
  if (!cap.isOpened())
    return -1;
  //while (1){
    cap >> frame;
    resize(frame, frame, Size(800, 600));
    GaussianBlur(frame, frame, Size(3, 3), 0, 0, BORDER_DEFAULT);
    cvtColor(frame, frame, CV_BGR2GRAY, 1);

    output = Mat::zeros(frame.size(), CV_8U);
    output = mag_thresh(frame, 3);
    //imshow("a", output);
    /*for (int i = 0; i < output.rows; i++)
      for (int j = 0; j < output.cols; j++){
        cout <<
      }*/
    cout << output << endl;
    //if (waitKey(30) == 27)
      //break;
  //}
  return 0;
}

Mat mag_thresh(Mat image, int sobel_kernel = 3){
  Mat sobelx, sobely, binary_output;

  // Take both Sobel x and y gradients
  Sobel(image, sobelx, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
  Sobel(image, sobely, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );

  // Calculate the gradient magnitude
  float scale_factor = 0;
  Mat gradmag = Mat::zeros(image.size(), CV_8U);
  unsigned char *input = (unsigned char*)(gradmag.data);
  unsigned char *in_sobelx = (unsigned char*)(sobelx.data);
  unsigned char *in_sobely = (unsigned char*)(sobely.data);
  unsigned char *out = (unsigned char*)(binary_output.data);

  for (int i = 0; i < gradmag.rows - 1; i++)
    for (int j = 0; j < gradmag.cols - 1; j++){
      //cout << sobelx.at<int>(j,i) << " ";
      //int x = in_sobelx[sobelx.step * j + i];
      //int y = in_sobely[sobely.step * j + i];
      uchar x = sobelx.at<uchar>(i,j);
      uchar y = sobely.at<uchar>(i,j);
      uchar k = sqrt(x*x + y*y);
      //cout << input[gradmag.step * j + i] << endl; // ko dc de sau ham ben duoi
      //cout << k << " ";
      gradmag.at<uchar>(i,j) = (uchar)k;
      //cout << gradmag.at<uchar>(j,i) << endl;
      if (scale_factor < k) scale_factor = k;
      //cout << i << " "<< j<<endl;
    }

    scale_factor /= 255;

  // Create a binary image of ones where threshold is met, zeros otherwise
  binary_output = Mat::zeros(gradmag.size(), CV_8U);;
  for (int i = 0; i < gradmag.rows - 1; i++)
    for (int j = 0; j < gradmag.cols - 1; j++){
      //gradmag.at<uchar>(j,i) /= scale_factor;
      //cout << gradmag.at<uchar>(i,j) << " ";
      if ((gradmag.at<uchar>(i,j) >= min_mag_thresh) && (gradmag.at<uchar>(i,j) <= max_mag_thresh))
        binary_output.at<uchar>(i,j) = 1;
      else
        binary_output.at<uchar>(i,j) = 0;
    }
    //cout << binary_output;
    return binary_output;
}

void dir_threshold(Mat image, Mat binary_output, int sobel_kernel = 3){
  // Calculate the x and y gradients
  Mat sobelx, sobely, absgraddir;
  Sobel(image, sobelx, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
  Sobel(image, sobely, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );

  // Take the absolute value of the gradient direction,
  // apply a threshold, and create a binary image result
  for (int i = 0; i < sobelx.cols; i++)
    for (int j = 0; j < sobelx.rows; j++){
      absgraddir.at<int>(i,j) = atan(abs(sobely.at<int>(i,j)) / abs(sobelx.at<int>(i,j)));
      //binary_output.at<double>
    }
}
