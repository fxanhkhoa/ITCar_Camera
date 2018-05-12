#include "sobel_thresh.h"

int main(int argc, char **argv)
{
  Mat frame;
  double *fit = new double[6];
  VideoCapture cap("../videos/clip1_FPT.mp4");
  if (!cap.isOpened())
    return -1;
  while (1) {
    cap >> frame;

    fit = sliding_window(frame);
    //cout << "fififi" << fit[3];

    if (waitKey(30) == 27)
      break;
  }
  return 0;
}
