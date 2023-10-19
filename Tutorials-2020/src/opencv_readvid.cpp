#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int main(int, char**){
    VideoCapture video_capture(0);
    if(!video_capture.isOpened()){return -1;}
    Mat edges;
    namedWindow("edges",1);
    while(true){
        Mat frame;
        video_capture >>frame;
        cvtColor(frame, edges, cv::COLOR_BGR2GRAY);
        imshow("edges",edges);
        imwrite("/home/mglocadmin/rob_work/src/Tutorials-2020/images/images_copy/last_rec.jpg",frame);
        if(waitKey(40)>=0){break;}
    }
    return 0;

}