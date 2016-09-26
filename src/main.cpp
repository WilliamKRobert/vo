#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/xfeatures2d.hpp"

#include "cal_pose.h"

using namespace std;
using namespace cv;

int main()
{
    Mat img1_l, img1_r, img2_l, img2_r;
    //img1_l = imread("/home/audren/vo/test_img/tsukuba_l.png", CV_LOAD_IMAGE_GRAYSCALE);
    //img1_r = imread("/home/audren/vo/test_img/tsukuba_r.png", CV_LOAD_IMAGE_GRAYSCALE);
    img1_l = imread("/home/audren/vo/test_img/img1_l.png", CV_LOAD_IMAGE_GRAYSCALE);
    img1_r = imread("/home/audren/vo/test_img/img1_r.png", CV_LOAD_IMAGE_GRAYSCALE);

    img2_l = imread("/home/audren/vo/test_img/img2_l.png", CV_LOAD_IMAGE_GRAYSCALE);
    img2_r = imread("/home/audren/vo/test_img/img2_r.png", CV_LOAD_IMAGE_GRAYSCALE);
 
    if(! img1_l.data || ! img1_r.data || !img2_l.data || !img2_r.data)
    {
        cout <<" Could not open or find the image" << endl;
        return -1;
    }
    
    cal_pose(img1_l, img1_r, img2_l, img2_r);

    return 0;
}
