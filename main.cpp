#include <opencv2/highgui.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
 
int main( int argc, char** argv ) {
  
  	VideoCapture cap;
	int deviceID = 0;
	int apiID = cv::CAP_ANY;
cap.open(deviceID + 0);
	//cap.open(deviceID + apiID);
	if (!cap.isOpened())
{
		cout << "can't open"<<endl;	
		return -1;
}
	// Device device;
	while (1)
	{
		Mat frame;
		cap >> frame;
		imshow("frame", frame);
		if (waitKey(30) >= 0) break;
	}
	return 0;
}
