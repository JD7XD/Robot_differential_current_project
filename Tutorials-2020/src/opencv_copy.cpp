#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int main(){
    Mat image;
    image = imread("/home/mglocadmin/rob_work/src/Tutorials-2020/images/tree.jpg",cv::IMREAD_COLOR);

    if(!image.data){
        cout<<"Could not find image\n";
        return -1;
    }

    namedWindow("IMG",cv::WINDOW_AUTOSIZE);
    imshow("IMG",image);
    imwrite("/home/mglocadmin/rob_work/src/Tutorials-2020/images/images_copy/copyt_tree.jpg",image);

    waitKey(0);
    return 0;

}