#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

RNG rng(12345);

int main( int argc, char** argv )
{
	VideoCapture cap(0);
	if (!cap.isOpened()){
		cout<<"error";
		return -1;
	}

	Mat edges, crop_img, gray, thresh;
	vector<vector<Point> > contours;
 	vector<Vec4i> hierarchy;
	namedWindow("edges",1);
	for (;;)
	{
	try
	{
		Mat frame;
		cap >> frame;
		//GpuMat d_frame;
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

		GaussianBlur(gray, edges, Size(15,15), 1.5, 1.5);

		//Canny(edges, edges,0 ,30, 3);

		threshold(gray, thresh, 150, 255, THRESH_BINARY);

		findContours(thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		if (contours.size() > 0)
		{
			Mat drawing = Mat::zeros( thresh.size(), CV_8UC3 );
			for (int i = 0; i < contours.size(); i++)
			{
				Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
				drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
			}


		imshow("edges",edges);
		imshow("frame",drawing);
		}
	}
	catch(const std::exception& e) {}
		if (waitKey(30) == 27) break;
	}
	return 0;
}
