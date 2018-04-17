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

#include "TK1_DRIVER/Controller.h"
#include "TK1_DRIVER/jetsonGPIO.h"

using namespace std;
using namespace cv;

#define MODE_CONTOUR 1
#define DEBUG 0

/******** Prototype ***********/
Point GetCenter(vector<Vec2f> lines);
void GetContours();
int mode_contour();
double getTheta(Point car, Point dst);
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
Mat frame;
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

  unsigned int value = high;

  if (!cap.isOpened())
    return -1;
  //_Controller->Handle(-45);
  while (1) {
    gpioGetValue(BTN1, &value);
    if (value == low) {
      //_Controller->Speed(100,100);
      //_Controller->Handle(90);
      mode = MODE_CONTOUR;
    }
    gpioGetValue(BTN2, &value);
    if (value == low) {
      mode = 0;
      _Controller->Speed(0, 0);
      _Controller->Handle(0);
    }
    if (mode == MODE_CONTOUR) {
      cap >> frame;
      motor_left = 100;
      motor_right = 100;
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
    resize(frame, frame, Size(800, 600));

    // Set region

    Rect roi;
    roi.x = 100;
    roi.y = 400;
    roi.width = 600;
    roi.height = 200;

    Point carPosition(300, 100);
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
    // imshow("frame1", frame);
    imshow("vanishing", crop_img);
    imshow("frame", thresh);

    double theta = getTheta(carPosition, Point(centerx, 50));
    // cout<< theta << endl;
    if (abs(theta) > 5) {
      _Controller->Handle(int(theta));

      if (theta > 0)
        motor_right -= theta;
      else if (theta < 0)
        motor_left -= theta;

      _Controller->Speed(motor_left, motor_right);
    } else {
      _Controller->Handle(0);
      _Controller->Speed(100, 100);
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
      if (area > 15000) {
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
#if DEBUG
    cout << endl;
#endif
    imshow("Contours", drawing);
  }
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
