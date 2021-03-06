//#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <math.h>
//#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/imgproc.hpp"

#include "../include/Controller.h"
#include "../include/jetsonGPIO.h"
#include "../include/Sign_regconize.h"

using namespace std;
using namespace cv;
using namespace cv::cuda;

#define MODE_CONTOUR 1
#define MODE_VANISHING 2
#define MODE_SIGN_REGCONIZE 3
#define DEBUG 0

/******** Prototype ***********/
Point GetCenter(vector<Vec2f> lines);
void GetContours();
int mode_contour();
int mode_vanishing();
int mode_sign_regconize();
double getTheta(Point car, Point dst);
Point GetCenter(vector<Vec2f> lines);
int getkey();
/*------ End Prototype ------*/

/******* Variables ********/
RNG rng(12345);
Mat edges, crop_img, gray, thresh;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
vector<Vec2f> lines;
Point center;
int mode = 0;
int sign_result = 0;
Mat frame;
bool flag = false, turned = false;
double centerx;
Controller *_Controller; // Motro and Servo controll variable
int motor_left, motor_right;
/*------ End Variables -------*/

int main(int argc, char **argv) {
  jetsonGPIONumber BTN1 = gpio160;
  jetsonGPIONumber BTN2 = gpio161;
  jetsonGPIONumber BTN3 = gpio163;
  jetsonGPIONumber BTN4 = gpio164;
  gpioExport(BTN1);
  gpioExport(BTN2);
  gpioExport(BTN3);
  gpioExport(BTN4);
  gpioSetDirection(BTN1, inputPin);
  gpioSetDirection(BTN2, inputPin);
  gpioSetDirection(BTN3, inputPin);
  gpioSetDirection(BTN4, inputPin);

  VideoCapture cap(0);
  _Controller = new Controller();

  // Wait for the push button to be pressed
  cout << "Please press the button! ESC key quits the program" << endl;
  flag = false;
  turned = false;
  unsigned int value = high;

  if (!cap.isOpened())
    return -1;
  //_Controller->Handle(-45);
  while (1) {
    gpioGetValue(BTN1, &value);
    if (value == low) {
      if (mode != MODE_CONTOUR)
        mode = MODE_CONTOUR;
      else{
        mode = 0;
        _Controller->Speed(0, 0);
        _Controller->Handle(0);
        flag = false;
      }
    }
    gpioGetValue(BTN2, &value);
    if (value == low) {
      if (mode != MODE_VANISHING)
        mode = MODE_VANISHING;
      else{
        mode = 0;
        _Controller->Speed(0, 0);
        _Controller->Handle(0);
      }
    }
    gpioGetValue(BTN3, &value);
    if (value == low){
      if (mode != MODE_SIGN_REGCONIZE)
        mode = MODE_SIGN_REGCONIZE;
      else{
        mode = 0;
        flag = false;
        turned = false;
        _Controller->Speed(0, 0);
        _Controller->Handle(0);
      }
    }
    gpioGetValue(BTN4, &value);
    if (value == low){
        mode = 0;
        _Controller->Speed(0, 0);
        _Controller->Handle(0);
    }
    if (mode == MODE_CONTOUR) {
      cap >> frame;
      motor_left = 100;
      motor_right = 100;
      turned = true;
      mode_contour();
    }
    else if (mode == MODE_VANISHING){
      cap >> frame;
      //mode_vanishing();
      //_Controller->Speed(20, 100);
      //_Controller->Handle(-60);
      mode_contour();
    }
    else if (mode == MODE_SIGN_REGCONIZE){
      cap >> frame;
      if (flag == false){
      //imshow("aaa",frame);
      //cout<<"a"<<endl;
        sign_result = mode_sign_regconize();
        if ((sign_result == 1) || (sign_result == 2))
            flag = true;
      }
      else
        mode_contour();
    }

    if (waitKey(30) == 27)
      break;
  }

  cout << "End" << endl;
  gpioUnexport(BTN1);
  gpioUnexport(BTN2);
  gpioUnexport(BTN3);
  gpioUnexport(BTN4);
  _Controller->Handle(0);
  _Controller->Speed(0, 0);
}

int mode_contour() {
  try {
    if (turned == false){
        //cout<<"in"<<endl;
    if (sign_result == 1){
        //trai
        _Controller->Handle(-45);
        _Controller->Speed(70,100);
        sleep(2);
        turned = true;
        motor_left = 100;
        motor_right = 100;
    }
    else if (sign_result == 2){
        //phai
        _Controller->Handle(45);
        _Controller->Speed(100,70);
        //for (long i = 0; i < 9999999; i++);
        sleep(2);
        turned = true;
        motor_left = 100;
        motor_right = 100;
    }
    }
    else{
    resize(frame, frame, Size(800, 600));

    // Set region

    Rect roi;
    roi.x = 000;
    roi.y = 500;
    roi.width = 800;
    roi.height = 100;

    Point carPosition(400, 100);
    Point prvPosition = carPosition;
    // Crop with ROI
    crop_img = frame(roi);
    cvtColor(crop_img, gray, COLOR_BGR2GRAY);

    GaussianBlur(gray, gray, Size(15, 15), 1.5, 1.5);

    // bitwise_not(gray, gray);

    threshold(gray, thresh, 150, 255, THRESH_BINARY_INV);

    // Canny(thresh, edges, 0, 30, 3);

    GetContours();

    // circle(crop_img, Point(x0,y0), 2, Scalar(5,255,100), CV_FILLED, 8,
    // 0);
    // circle(crop_img, center, 3, Scalar(168, 1, 170), CV_FILLED, 8, 0);
     imshow("frame1", frame);
    imshow("vanishing", crop_img);
    imshow("frame", thresh);

    double theta = getTheta(carPosition, Point(centerx, 50));
    // cout<< theta << endl;
    if (abs(theta) > 5) {
      _Controller->Handle(int(theta));
      motor_left = 100;
      motor_right = 100;
      if (theta > 0)
        motor_right -= theta;
      else if (theta < 0)
        motor_left -= theta;

      _Controller->Speed(motor_left, motor_right);
    } else {
      _Controller->Handle(0);
      _Controller->Speed(100, 100);
    }
   }
  } catch (const std::exception &e) {
  }
  return 0;
}

void GetContours() {
  bool check_left, check_right;
  findContours(thresh, contours, hierarchy, CV_RETR_TREE,
               CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

#if DEBUG
  cout << thresh.rows << " " << thresh.cols << endl;
#endif

// Get Moment
#if DEBUG
  long max = 0;
#endif
  int dem = 0;
  centerx = 0; // Reset centerx
  Scalar color =
      Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
  if (contours.size() > 0) {
    vector<Moments> mu(contours.size());
    Mat drawing = Mat::zeros(thresh.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++) {
      double area = contourArea(contours[i]);
      double arclength = arcLength(contours[i], true);
      if (area > 25000) {
        dem++; //(Add number of contour)
        // Add moment point
        mu[i] = moments(contours[i], false);

#if DEBUG
        cout << i << ":" << area << " length:" << arclength << " ";
        if (max < area)
          max = area;
#endif
        // MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());

        // Get Mass Center
        vector<Point2f> mc(contours.size());
        mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
        centerx += mc[i].x;
        circle(drawing, mc[i], 4, color, -1, 8, 0);

        // double perimeter = Imgproc.arcLength(contour2f, true);
        // Found squareness equation on wiki...
        // https://en.wikipedia.org/wiki/Shape_factor_(image_analysis_and_microscopy)
        // double squareness = 4 * Math.PI * area / Math.pow(perimeter, 2);

        // if (squareness )
        // if ( hierarchy[i][4] == -1 ) {

        // Check each point
        check_left = false;
        check_right = false;
        /*for (int j = 0; j < contours[i].size(); j++) {
        // cout<< i << ": "<< contours[i][j].x <<endl;
        // circle(drawing,contours[i][j],1,cv::Scalar(0,0,255));
        if ((contours[i][j].x < 296))
          check_left = true;
        else if (contours[i][j].x > 304)
          check_right = true;
      }
      #if DEBUG
        cout << i <<" "<< dem <<endl;
      #endif
      */ // if (area > 30000)
        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
        //}
      }
    }
    centerx = centerx / dem;
    circle(drawing, Point(centerx, 50), 4, color, -1, 8, 0);
// cout<<" max" << max <<endl;

    imshow("Contours", drawing);
#if DEBUG
#endif

  }
}

int mode_vanishing(){
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

  HoughLines(edges, lines, 1, CV_PI / 180, 55, 0, 0);
  if (lines.size() > 0) {
    center = GetCenter(lines);
  }
  circle(crop_img, center, 5, Scalar(168, 1, 170), CV_FILLED, 8, 0);
  imshow("vanishing", crop_img);
}

Point GetCenter(vector<Vec2f> lines){
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

      // Calreturn number_result;culate Intersection
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

int mode_sign_regconize(){
  Mat result;
  int number_result;
  if(recognizeBlueSign(frame, result) == 1){
      cout<<"trai"<<endl;
      number_result = 1;
  }
  else if(recognizeBlueSign(frame, result)==2){
      cout<<"phai"<<endl;
      number_result = 2;
  }
  imshow("frame", frame);
  return number_result;
}


/*****************************************************************/

int getkey() {
  int character;
  struct termios orig_term_attr;
  struct termios new_term_attr;

  /* set the terminal to raw mode */
  tcgetattr(fileno(stdin), &orig_term_attr);
  memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
  new_term_attr.c_lflag &= ~(ECHO | ICANON);
  new_term_attr.c_cc[VTIME] = 0;
  new_term_attr.c_cc[VMIN] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

  /* read a character from the stdin stream without blocking */
  /*   returns EOF (-1) if no character is available */
  character = fgetc(stdin);

  /* restore the original terminal attributes */
  tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

  return character;
}

double getTheta(Point car, Point dst) {
  if (dst.x == car.x)
    return 0;
  if (dst.y == car.y)
    return (dst.x < car.x ? -90 : 90);
  // cout << "center x"<< dst.x << " ";
  double pi = acos(-1.0);
  double dx = dst.x - car.x;
  double dy = car.y - dst.y; // image coordinates system: car.y > dst.y
#if DEBUG
  cout << dx << " " << dy << endl;
#endif
  if (dx < 0)
    return -atan(-dx / dy) * 180 / pi;
  return atan(dx / dy) * 180 / pi;
}
