#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include "TK1_DRIVER/Controller.h"

using namespace std;
using namespace cv;

/******** Prototype ***********/
Point GetCenter(vector<Vec2f> lines);
void GetContours();
/*------ End Prototype ------*/

RNG rng(12345);
Mat edges, crop_img, gray, thresh;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
vector<Vec2f> lines;
Point center;
Mat frame;

int main(int argc, char **argv) {
  VideoCapture cap("videos/clip1_FPT.mp4");
  if (!cap.isOpened())
    return -1;

  //namedWindow("edges", 1);

  for (;;) {
    try {

      cap >> frame;
      resize(frame, frame, Size(800, 600));
      // Set region

      Rect roi;
      roi.x = 100;
      roi.y = 200;
      roi.width = 600;
      roi.height = 200;

      Mat element = getStructuringElement(MORPH_CROSS, Size(5, 5), Point(2, 2));

      // Crop with ROI
      crop_img = frame(roi);
      cvtColor(crop_img, gray, COLOR_BGR2GRAY);

      GaussianBlur(gray, gray, Size(15, 15), 1.5, 1.5);

      // bitwise_not(gray, gray);

      threshold(gray, thresh, 150, 255, THRESH_BINARY_INV);

      Canny(thresh, edges, 125, 255);
      dilate(edges, edges, element,Point(-1,-1));
			erode(edges, edges, element, Point(-1, -1));

      HoughLines(edges, lines, 1, CV_PI / 180, 50, 0, 0);
      if (lines.size() > 0) {
        center = GetCenter(lines);
        GetContours();
      } else {
        GetContours();
      }
      // circle(crop_img, Point(x0,y0), 2, Scalar(5,255,100), CV_FILLED, 8,
      // 0);
      circle(crop_img, center, 3, Scalar(168, 1, 170), CV_FILLED, 8, 0);
      imshow("frame", frame);
      imshow("vanishing", crop_img);
      imshow("frame", thresh);
    } catch (const std::exception &e) {
    }
    if (waitKey(30) == 27)
      break;
  }
  return 0;
}

Point GetCenter(vector<Vec2f> lines) {
  double Centerx = 0;
  int count = 0;
  // cout <<"size:"<< lines.size()<< endl;
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
        // cout << "parallel" << endl;
      } else {
        count++;
        double xi = (rho1 * b2 - rho2 * b1) / determinant;
        Centerx += xi;
        double yi = (a1 * rho2 - a2 * rho1) / determinant;
        circle(crop_img, Point(xi, 50), 2, Scalar(5, 255, 100), CV_FILLED, 8,
               0);
      }

      double x0 = a1 * rho1, y0 = b1 * rho1;
      // Phuong trinh ax + by = c voi a = cos(theta) va b = sin(theta)

      pt1.x = cvRound(x0 + 1000 * (-b1));
      pt1.y = cvRound(y0 + 1000 * (a1));
      pt2.x = cvRound(x0 - 1000 * (-b1));
      pt2.y = cvRound(y0 - 1000 * (a1));

      line(crop_img, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
    }
  }
  // cout<<"Count:"<<count<<endl;
  return Point(Centerx / count, 50);
}

void GetContours() {
  bool check_left, check_right;
  findContours(thresh, contours, hierarchy, CV_RETR_TREE,
               CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
  // cout << thresh.rows << " " << thresh.cols << endl;
  if (contours.size() > 0) {
    Mat drawing = Mat::zeros(thresh.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++) {
      double area = contourArea(contours[i]);
      double arclength = arcLength(contours[i], true);
      // cout << i <<":" << area << " length:"<< arclength << " ";
      // MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());

      // double perimeter = Imgproc.arcLength(contour2f, true);
      // Found squareness equation on wiki...
      // https://en.wikipedia.org/wiki/Shape_factor_(image_analysis_and_microscopy)
      // double squareness = 4 * Math.PI * area / Math.pow(perimeter, 2);

      // if (squareness )
      // if ( hierarchy[i][4] == -1 ) {
      Scalar color =
          Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
      // Check each point
      check_left = false;
      check_right = false;
      for (int j = 0; j < contours[i].size(); j++) {
        // cout<< i << ": "<< contours[i][j].x <<endl;
        // circle(drawing,contours[i][j],1,cv::Scalar(0,0,255));
        if ((contours[i][j].x < 296))
          check_left = true;
        else if (contours[i][j].x > 304)
          check_right = true;
      }
      // cout << i <<" "<< dem <<endl;
      if ((check_left) && (check_right))
        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
      //}
    }
    //cout << endl;
    imshow("Contours", drawing);
  }
}
