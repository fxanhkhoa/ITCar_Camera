#include <opencv2/highgui.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;
using namespace std;
 
int main( int argc, char** argv ) {
  
  	VideoCapture cap(0);
	int deviceID = 0;
	int apiID = cv::CAP_ANY;
	//cap.open(deviceID + 0);
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
		resize(frame, frame, Size(800, 600));
		imshow("frame", frame);
		if (waitKey(30) == 27) break;
	}
	return 0;
}
