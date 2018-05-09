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

int main(int argc, char **argv) {

  Mat src, src_gray;
  Mat frame, thresh;
  Mat grad;
  Mat drawing;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  RNG rng(12345);
  // char *window_name = "Sobel Demo - Simple Edge Detector";
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;

  int c;

  VideoCapture cap("videos/Line1.mp4");
  if (!cap.isOpened())
    return -1;

  while (1) {
    cap >> frame;
    resize(frame, frame, Size(800, 600));

    GaussianBlur(frame, frame, Size(3, 3), 0, 0, BORDER_DEFAULT);

    /// Convert it to gray
    cvtColor(frame, src_gray, CV_BGR2GRAY);

    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    // Gradient X
    //Scharr(src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT);
    Sobel(src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(grad_x, abs_grad_x);

    /// Gradient Y
    //Scharr(src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT);
    Sobel(src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(grad_y, abs_grad_y);

    /// Total Gradient (approximate)
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
    imshow("sobel", grad);

    // contour

    // bitwise_not(gray, gray);

    // adaptiveThreshold(grad , thresh, 255, ADAPTIVE_THRESH_MEAN_C,
    // CV_THRESH_BINARY, 13, 0);
    threshold(grad, thresh, 100, 255, THRESH_BINARY);

    imshow("thresh", thresh);

    findContours(thresh, contours, hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    if (contours.size() > 0) {
      Mat drawing = Mat::zeros(thresh.size(), CV_8UC3);
      for (int i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                              rng.uniform(0, 255));
        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
      }
      imshow("Contours", drawing);
      if (waitKey(30) == 27)
        break;
    }
  }
    return 0;
}
