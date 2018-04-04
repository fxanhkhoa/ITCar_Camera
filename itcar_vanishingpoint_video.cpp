#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

RNG rng(12345);

int main(int argc, char **argv) {
  VideoCapture cap("line_video.mp4");
  if (!cap.isOpened())
    return -1;

  Mat edges, crop_img, gray, thresh;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  vector<Vec2f> lines;

  namedWindow("edges", 1);

  for (;;) {
    try {
      Mat frame;
      cap >> frame;
      resize(frame, frame, Size(800, 600));
      // Set region

      Rect roi;
      roi.x = 100;
      roi.y = 100;
      roi.width = 600;
      roi.height = 200;

      // Crop with ROI
      crop_img = frame(roi);
      cvtColor(crop_img, gray, COLOR_BGR2GRAY);

      GaussianBlur(gray, edges, Size(15, 15), 1.5, 1.5);

      threshold(gray, thresh, 150, 255, THRESH_BINARY);

      Canny(thresh, edges, 0, 30, 3);

      HoughLines(edges, lines, 1, CV_PI / 180, 100, 0, 0);

      for (size_t i = 0; i < lines.size() - 1; i++) {
        for (size_t j = i + 1; j < lines.size(); j++) {
          float rho1 = lines[i][0], theta1 = lines[i][1];
          float rho2 = lines[j][0], theta2 = lines[j][1];
          Point pt1, pt2;
          double a1 = cos(theta1), b1 = sin(theta1);
          double a2 = cos(theta2), b2 = sin(theta2);

          // Calculate Intersection
          double determinant = a1 * b2 - a2 * b1;
          if (determinant == 0) {
            // parallel
            cout << "parallel" << endl;
          } else {
            double xi = (rho1 * b2 - rho2 * b1) / determinant;
            double yi = (a1 * rho2 - a2 * rho1) / determinant;
            circle(crop_img, Point(xi, 50), 2, Scalar(5, 255, 100), CV_FILLED,
                   8, 0);
          }

          double x0 = a1 * rho1, y0 = b1 * rho1;
          // Phuong trinh ax + by = c voi a = cos(theta) va b = sin(theta)

          pt1.x = cvRound(x0 + 1000 * (-b1));
          pt1.y = cvRound(y0 + 1000 * (a1));
          pt2.x = cvRound(x0 - 1000 * (-b1));
          pt2.y = cvRound(y0 - 1000 * (a1));

          line(crop_img, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
          // circle(crop_img, Point(x0,y0), 2, Scalar(5,255,100), CV_FILLED, 8,
          // 0);
          // circle(crop_img, Point(a,b), 2, Scalar(5,100,100), CV_FILLED, 8,
          // 0);
        }
      }

      imshow("vanishing", crop_img);
      // imshow("frame",drawing);
    } catch (const std::exception &e) {
    }
    if (waitKey(30) >= 0)
      break;
  }
  return 0;
}
