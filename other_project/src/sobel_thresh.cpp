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
#include <cmath>

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
// Choose the number of sliding windows
int nwindows = 9;

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

int main(int argc, char **argv) {
  Mat frame, output;
  VideoCapture cap("../videos/clip1_FPT.mp4");
  if (!cap.isOpened())
    return -1;
  while (1) {
    cap >> frame;
    resize(frame, frame, Size(1280, 720));
    GaussianBlur(frame, frame, Size(3, 3), 0, 0, BORDER_DEFAULT);
    cvtColor(frame, frame, CV_BGR2GRAY, 1);

    output = Mat::zeros(frame.size(), CV_8U);
    output = thresholding(frame, 50, 255, 0, 25, 50, 255, 0.7, 1.3, 113, 255,
                          234, 255, 15, 250);

    // output = abs_sobel_thresh(frame, 1, 3, 150,255);
    // output = dir_threshold(frame, 3, 0, M_PI/2);
    //output = mag_thresh(frame, 3, 200, 255);

    frame = get_perspective(frame, output);
    int histogram[1200] = {0};

    for (int i = 0; i < 1080; i++){
      for (int j = frame.rows / 2; j < frame.rows; j++){
        if (frame.at<uchar>(j,i) == 255){
          histogram[i]++;
        }
      }
    }
    int leftx_base, rightx_base, max = 0;
    // Get left base
    for (int i = 0; i < 1080/2; i ++){
      if (max < histogram[i]){
        max = histogram[i];
        leftx_base = i;
      }
    }

    //Get right base
    max = 0;
    for (int i = 1080 / 2; i < 1080; i++){
      if (max < histogram[i]){
        max = histogram[i];
        rightx_base = i;
      }
    }
    int window_height = frame.rows / nwindows;
    int nonzerox[frame.rows*frame.cols];
    int nonzeroy[frame.rows*frame.cols];
    int nonzero_count = 0;

    for (int i = 0; i < frame.rows; i++)
      for (int j = 0; j < frame.cols; j++){
        if (frame.at<uchar>(i,j) == 255){
          nonzerox[nonzero_count] = j;
          nonzeroy[nonzero_count] = i;
          nonzero_count++;
        }
      }

    int margin = 50;
    int minpix = 50;
    int leftx_current, rightx_current;
    leftx_current = leftx_base;
    rightx_current = rightx_base;
    vector<Point> left_lane_inds;// Put here for concate with all windows
    vector<Point> right_lane_inds;// Put here for concate with all windows
    double leftx[8000], lefty[6000], rightx[8000], righty[6000];
    int count_nozeroleft = 0, count_nozeroright = 0;

    for (int i = 0; i < nwindows; i++){
      int win_y_low, win_y_high, win_xleft_low, win_xleft_high, win_xright_low, win_xright_high;

      win_y_low = frame.rows - (i * window_height);
      win_y_high = frame.rows - ((i + 1) * window_height);

      win_xleft_low = leftx_current - margin;
      win_xleft_high = leftx_current + margin;
      win_xright_low = rightx_current - margin;
      win_xright_high = rightx_current + margin;
      rectangle(frame, Point(win_xleft_low, win_y_low), Point(win_xleft_high, win_y_high), Scalar(255,255,255));
      rectangle(frame, Point(win_xright_low, win_y_low), Point(win_xright_high, win_y_high), Scalar(255,255,255));

      //Identify the nonzero pixels in x and y within the window
      //Point left_lane_inds[1000], right_lane_inds[1000];
      int count_left_inds = 0, count_right_inds = 0;
      long Sum_Left_x = 0, Sum_Right_x = 0;

      for (int i = win_y_high; i <= win_y_low; i++)
        for (int j = win_xleft_low; j <= win_xleft_high; j++){
          if (frame.at<uchar>(i,j) == 255){
            left_lane_inds.push_back(Point(i,j));
            leftx[count_nozeroleft] = j;
            lefty[count_nozeroleft] = i;
            count_nozeroleft++;
            count_left_inds++;
            Sum_Left_x += j;
          }
        }

      for (int i = win_y_high; i <= win_y_low; i++)
        for (int j = win_xright_low; j <= win_xright_high; j++){
          if (frame.at<uchar>(i,j) == 255){
            right_lane_inds.push_back(Point(i,j));
            rightx[count_nozeroright] = j;
            righty[count_nozeroright] = i;
            count_nozeroright++;
            count_right_inds++;
            Sum_Right_x += j;
          }
        }

      // Repositioning center point
      if (count_left_inds > minpix){
        leftx_current = Sum_Left_x / count_left_inds;
      }
      if (count_right_inds > minpix){
        rightx_current = Sum_Right_x / count_right_inds;
      }
    }
    //cout<<"OK here" << count_nozeroright << endl;
    polyfit(leftx, lefty, count_nozeroleft, 2);

    imshow("frame", frame);
    if (waitKey(30) == 27)
      break;
  }
  return 0;
}

double *polyfit(double x[], double y[], int count, int degree){
  int N = count, n = degree, i, j, k;
  double X[2*n + 1];

  for ( i = 0; i < 2*n + 1; i++){
    X[i] = 0;
    for ( j = 0; j < N; j++){
      X[i] = X[i] + pow(x[j], i);
    }
  }

  double B[n + 1][n + 2];
  double a[ n + 1];
  for ( i = 0; i <= n; i++){
    for ( j = 0; j <= n; j++){
      B[i][j] = X[i + j];
    }
  }

  double Y[n + 1];
  for ( i = 0; i <= n+1; i++){
    Y[i] = 0;
    for ( j = 0; j < N; j++){
      Y[i] = Y[i] + pow(x[j], i) * y[j];
    }
  }
  for ( i = 0; i <= n; i++){
    B[i][n + 1] = Y[i];
  }

  n = n + 1;
  cout<<"\nThe Normal(Augmented Matrix) is as follows:\n";
    for (i=0;i<n;i++)            //print the Normal-augmented matrix
    {
        for (j=0;j<=n;j++)
            cout<<B[i][j]<<" ";
        cout<<"\n";
    }

    for (i=0;i<n;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
          for (k=i+1;k<n;k++)
              if (B[i][i]<B[k][i])
                  for (j=0;j<=n;j++)
                  {
                      double temp=B[i][j];
                      B[i][j]=B[k][j];
                      B[k][j]=temp;
                  }

      for (i=0;i<n-1;i++)            //loop to perform the gauss elimination
          for (k=i+1;k<n;k++)
              {
                  double t= (B[k][i]) / (B[i][i]);
                  for (j=0;j<=n;j++)
                      B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
              }

              for (i=n-1;i>=0;i--)                //back-substitution
            {                        //x is an array whose values correspond to the values of x,y,z..
                a[i]=B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
                for (j=0;j<n;j++)
                    if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                        a[i]=a[i]-B[i][j]*a[j];
                a[i]= a[i]/ (B[i][i]);            //now finally divide the rhs by the coefficient of the variable to be calculated
            }
            cout<<"\nThe values of the coefficients are as follows:\n";
            for (i=0;i<n;i++)
                cout<<"x^"<<i<<"="<<a[i]<<endl;            // Print the values of x^0,x^1,x^2,x^3,....
            cout<<"\nHence the fitted Polynomial is given by:\ny=";
            for (i=0;i<n;i++)
                cout<<" + ("<<a[i]<<")"<<"x^"<<i;
            cout<<"\n";
    return a;
}

Mat get_perspective(Mat frame, Mat src){
  int offset = 0;
  Mat mask = Mat::zeros(frame.size(), CV_8U);
  Point pts_dst[4] = {
    Point(0 + offset, frame.rows),
    Point(frame.cols / 2.9, frame.rows / 1.9),
    Point(frame.cols / 1.5, frame.rows / 1.9),
    Point(frame.cols, frame.rows),
  };

  Rect roi;
  roi.x = 100;
  roi.y = frame.rows/1.9;
  roi.width = 1280 - 2*100;
  roi.height = frame.rows - roi.y;

  Mat crop_img;
  int off = 10;
  Point2f inputQuad[4], outputQuad[4];
  // lay tam cua tung line giong len ra output (input la )
  /*
  inputQuad[0] = Point2f(580, 460);
  inputQuad[1] = Point2f(205, 720);
  inputQuad[2] = Point2f(1110, 720);
  inputQuad[3] = Point2f(703, 460);

  outputQuad[0] = Point2f(320, 0);
  outputQuad[1] = Point2f(320, 720);
  outputQuad[2] = Point2f(960, 720);
  outputQuad[3] = Point2f(960, 0);
  */

  inputQuad[0] = Point2f(0 + offset, frame.rows);
  inputQuad[1] = Point2f(frame.cols / 2.9, frame.rows / 2.5);
  inputQuad[2] = Point2f(frame.cols / 1.5, frame.rows / 2.5);
  inputQuad[3] = Point2f(frame.cols, frame.rows);

  outputQuad[0] = Point2f((frame.cols / 2.9 + 0 + offset) / 2, frame.rows);
  outputQuad[1] = Point2f((frame.cols / 2.9 + 0 + offset) / 2, 0);
  outputQuad[2] = Point2f((frame.cols / 1.5 + frame.cols) / 2, 0);
  outputQuad[3] = Point2f((frame.cols / 1.5 + frame.cols) / 2, frame.rows);

  fillConvexPoly(mask, pts_dst, 4, Scalar(255));
  bitwise_and(mask, src, frame);
  Mat trans_mat33 = Mat::zeros(frame.size(), CV_64F);
  trans_mat33 = getPerspectiveTransform(inputQuad, outputQuad);
  warpPerspective(frame, frame, trans_mat33, frame.size(), cv::INTER_LINEAR);

  crop_img = frame(roi);
  return crop_img;
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
