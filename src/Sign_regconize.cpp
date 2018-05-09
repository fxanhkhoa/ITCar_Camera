#include "../include/Sign_regconize.h"

//Sử dụng đặc trưng HOG (Histogram of gradient)
HOGDescriptor hog(
        Size(36,36), //winSize
        Size(6,6), //blocksize
        Size(6,6), //blockStride,
        Size(6,6), //cellSize,
                 9, //nbins,
                  1, //derivAper,
                 -1, //winSigma,
                  0, //histogramNormType,
                0.2, //L2HysThresh,
                  0,//gammal correction,
                  64,//nlevels=64
                  1);

/***********************************Load file svm đã training********************************************/

Ptr<SVM> svm = Algorithm::load<SVM>("train.xml");             //Load bien xanh

/********************************************************************************************************/

/************************************Biến đổi hình thái học**********************************************/
void Erosion(Mat src, Mat &dst, int erosion_elem, int erosion_size){
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  cv::erode( src, dst, element );
}

void Dilation(Mat src, Mat &dst, int dilation_elem, int dilation_size ){
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );

  /// Apply the dilation operation
  cv::dilate(src, dst, element );
}

void BinaryOpening(Mat src, Mat &dst, int erosion_size, int dilation_size){
    Mat elementErode = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );

    Mat elemenDilate = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         Point( dilation_size, dilation_size ) );

    cv::erode(src,dst,elementErode);
    cv::dilate(dst, dst, elemenDilate );
}

void BinaryClosing(Mat src, Mat &dst, int dilation_size, int erosion_size){
    Mat elementErode = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );

    Mat elemenDilate = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         Point( dilation_size, dilation_size ) );
    cv::dilate(src, dst, elemenDilate );
    cv::erode(dst,dst,elementErode);
}

/***************************************************************************************************/


/****************************************Lọc diện tích**********************************************/

void removeSmallOject(Mat src, Mat &dst, int min_size, int max_size){
    vector<vector<Point> > contours;
    cv::findContours(src, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

    src.copyTo(dst);

    for(int i=0;i<contours.size();i++){
        double S=contourArea(contours[i]);
        //Kiem tra dieu kien neu nho hon min_size hoac lon hon max_size thi dua cac diem anh trong contour ve 0 (mau den)
        if (fabs(S) <= min_size)
            cv::drawContours(dst, contours, i, Scalar::all(0), CV_FILLED);
        else if (fabs(S) > max_size)
            cv::drawContours(dst, contours, i, Scalar::all(0), CV_FILLED);
        else
            continue;
    }
}

/***************************************************************************************************/

/******************************************Phân đoạn màu********************************************/

void blueColorSegmentation(Mat src, Mat &dst){
    Mat hsv_frame;
    cvtColor(src, hsv_frame, CV_BGR2HSV);

    for(int i=0; i<hsv_frame.rows; i++)
        for(int j=0; j<hsv_frame.cols; j++){
            Vec3b& v=hsv_frame.at<Vec3b>(i,j);
            if((v[0]<Hblue_min) || (v[0]>Hblue_max))
                continue;
            else{
                if((v[2]>Vblue_max) || (v[2]<Vblue_min) || (v[1]<Sblue_min))
                    continue;
                else if(v[1] > Sblue_max)
                    dst.at<uchar>(i,j)=255;
                else
                    continue;
            }
        }
    removeSmallOject(dst, dst, MIN_OBJECT_COLOR_SEG, MAX_OBJECT_COLOR_SEG);
}

void signAreaDetection(Mat src, Mat &dst){
    Mat blueImg[3];
    dst = Mat::zeros(src.size(), src.type());
    cv::split(src, blueImg);
    //imshow("blue", blueImg[0]);
    for(int i=0; i<blueImg[0].rows; i++)
        for(int j=0; j<blueImg[0].cols; j++){
            if(blueImg[0].at<uchar>(i,j)>50 && blueImg[0].at<uchar>(i,j) < 200)
                dst.at<uchar>(i,j) = 255;
        }
    //imshow("dst", dst);
}

/*****************************************************************************************************************/

/************************************Lọc diện tích và tỉ lệ cạnh của đối tượng************************************/

void areaAndRatio(Mat src, Mat &dst){
    cv::dilate(src, src, 5);
    cv::erode(src, src, 5);
    vector<vector<Point> > contours;
    cv::findContours(src, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

    dst=Mat::zeros(Size(src.cols, src.rows), src.type());

    for(int i=0;i<contours.size();i++){
        double S=contourArea(contours[i]);
        if ((S > MIN_OBJECT_AREA_SIGN) && (S < MAX_OBJECT_AREA_SIGN)){
            Rect rect=boundingRect(Mat(contours[i]));
            double ratio= (double)rect.width/ (double)rect.height;
            //Trường hợp biển nằm ngoài khung hình
            /*if((rect.x>15)&&(rect.y>15) && (rect.x+rect.width<src.cols-15)
                &&(rect.y+rect.height<src.rows-15) && (ratio>ratio_min) && (ratio<ratio_max))*/
            if((ratio>ratio_min) && (ratio<ratio_max)){
                cv::drawContours(dst, contours, i, Scalar::all(255), CV_FILLED);
            }
            else
                continue;
        }
        else
            continue;
    }
}

/************************************************************************************************************/

/*****************************************Phát hiện hình Ellipse*********************************************/

bool detectShape(Mat src, Rect rect){
    Mat temp;
    src(rect).copyTo(temp);
    //imshow("temp", temp);

    int filled_blob_height, filled_blob_width;
    float f_x_y = 0;
    float micro_20 = 0;	//dùng để tính I
    float micro_02 = 0;	//dùng để tính I
    float micro_11 = 0;	//dùng để tính I
    float micro_00 = 0;	//dùng để tính I
    float I = 0;
    float E = 0;

    filled_blob_height = temp.rows;
    filled_blob_width = temp.cols;

    int x_center = filled_blob_width / 2;
    int y_center = filled_blob_height / 2;

    for (int y = 0; y < filled_blob_height; y++){
        for (int x = 0; x < filled_blob_width; x++){
            f_x_y = temp.at<uchar>(x, y)/255;
            micro_20 += (x - x_center) * (x - x_center) * f_x_y;
            micro_02 += (y - y_center) * (y - y_center) * f_x_y;
            micro_11 += (x - x_center) * (y - y_center) * f_x_y;
            micro_00 += f_x_y;
        }
    }

    //tinh I1
    I = (micro_20 * micro_02 - micro_11 * micro_11) / (micro_00 * micro_00 * micro_00 * micro_00);

    //tinh E ap dung cong thuc
    if (I <= (1 / 16 / 3.14159 / 3.14159))
        E = 16 * 3.14159 * 3.14159 * I;
    else
        E = 1 / (16 * 3.14159 * 3.14159 * I);

    cout<<E<<endl;
    if ((E >= E_min) && (E <= E_max))
        return true; //circle dung
    else
        return false; //nothing
}

void detectEllipse(Mat src, Mat &dst){
    vector<vector<Point> > contours;
    vector<vector<Point> > temp;
    Mat x;

    src.copyTo(x);

    cv::findContours(src, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

    temp.resize(contours.size());

    dst=Mat::zeros(Size(src.cols, src.rows), CV_8UC1);

    for(int i=0;i<contours.size();i++){
        //convexHull(contours[i], temp1[i]);
        cv::approxPolyDP(Mat(contours[i]), temp[i], 0.02*arcLength(contours[i], true), true);
        if(temp[i].size()>max_vertices){
            double S = contourArea(contours[i]);
            Rect r = boundingRect(contours[i]);
            int radius = r.width / 2;
            float ff=abs(1 - (double)(S / (CV_PI * (radius*radius))));
            //cout<<ff<<endl;

            if ((abs(1 - ((double)r.width / r.height)) <= ratio_shape_max) && (abs(1 - (double)(S / (CV_PI * (radius*radius)))) <= ratio_area_max)){
                //imshow("x", x);
                if(detectShape(x, r))
                    cv::drawContours(dst, contours, i, Scalar::all(255), CV_FILLED);
                else
                    continue;
            }
            else
                continue;
        }
    }
}

/***************************************************************************************************************************/

/************************************************Nhận diện loại biển báo theo yêu cầu***************************************/

int trafficBlueSignRecognize(Mat src, Rect rect){

    /*0-> Bien 02
     *1-> Bien 08
     *2-> Bien 03
     *3-> Negative
     *4-> Bien 09  */

    Mat temp;
    src(rect).copyTo(temp);
    vector<float> feature;
    resize(temp, temp, Size(36,36));
    cvtColor(temp, temp, CV_BGR2GRAY);
    hog.compute(temp, feature);
    Mat data = Mat(1 , feature.size(), CV_32FC1);
    for(size_t t=0; t<feature.size(); t++)
        data.at<float>(0, t) = feature.at(t);
    //cout<<svm->predict(data)<<endl;
    return svm->predict(data);
}

int recognizeBlueSign(Mat &src, Mat& dst){
    dst=Mat::zeros(Size(src.cols, src.rows), CV_8UC1);
    vector<vector<Point> > contours, contours1;

    blueColorSegmentation(src, dst);
    //imshow("Blue after segment", dst);
    //GaussianBlur(dst, dst, Size(5, 5), 0);
    cv::medianBlur(dst, dst, 5);
    //BinaryClosing(dst, dst, 9, 3);
    //Dilation(dst, dst, 2, 3);
    //Erosion(dst, dst, 2, 3);


    //imshow("Blue after morphy", dst);
    cv::findContours(dst, contours1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    dst = Mat::zeros(dst.size(), dst.type());
    for(size_t i=0; i<contours1.size(); i++)
        cv::drawContours(dst, contours1, i, cv::Scalar(255), CV_FILLED);
    //imshow("Blue after draw", dst);
    //Dilation(dst, dst, 2, 3);
    //Erosion(dst, dst, 2, 3);

    //imshow("Blue after Morphy2", dst);
    areaAndRatio(dst,dst);
    Dilation(dst, dst, 2, 5);
    Erosion(dst, dst, 2, 5);
     //imshow("Blue after filter", dst);
    //medianBlur(dst, dst, 5);
    detectEllipse(dst, dst);
    imshow("after detect ellipse", dst);
    cv::findContours(dst, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    for(int i=0;i<contours.size();i++){
        Rect r = boundingRect(Mat(contours[i]));
        if(trafficBlueSignRecognize(src, r)==0){
            return 1;
            //rectangle(src, r, Scalar(0,0,255),1,8,0);
            //putText(src, "rẽ trái", Point(r.x, r.y), FONT_HERSHEY_COMPLEX, 0.8, Scalar(0,255,0));
        }
        if(trafficBlueSignRecognize(src, r)==1){
            return 2;
            //rectangle(src, r, Scalar(0,0,255),1,8,0);
            //putText(src, "rẽ phải", Point(r.x, r.y), FONT_HERSHEY_COMPLEX, 0.8, Scalar(0,255,0));
        }
    }
    return 0;
}
