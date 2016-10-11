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

    img1_l = imread("../test_img/img1_l.png", CV_LOAD_IMAGE_GRAYSCALE);
    img1_r = imread("../test_img/img1_r.png", CV_LOAD_IMAGE_GRAYSCALE);

    img2_l = imread("../test_img/img2_l.png", CV_LOAD_IMAGE_GRAYSCALE);
    img2_r = imread("../test_img/img2_r.png", CV_LOAD_IMAGE_GRAYSCALE);
 
    if(! img1_l.data || ! img1_r.data || !img2_l.data || !img2_r.data)
    {
        cout <<" Could not open or find the image" << endl;
        return -1;
    }
    
    Eigen::MatrixXf P1(3, 4), P2(3, 4);
    P1 << 718.8560,        0, 607.1928,         0,
                 0, 718.8560, 185.2157,         0,
                 0,        0,   1.0000,         0;
    P2 << 718.8560,        0, 607.1928, -386.1448,
                 0, 718.8560, 185.2157,         0,
                 0,        0,   1.0000,         0;

	Mat R = Mat(3, 3, CV_32F, cvScalar(0.));
   	Mat t = Mat(3, 1, CV_32F, cvScalar(0.));
	int method = 1;
    cal_pose( img1_l, img1_r, img2_l, img2_r, P1, P2, R, t, method );
	
	cout <<endl;
	cout <<"Rotation Matrix: " <<endl <<R <<endl;
	cout <<endl;
	cout <<"Translation vector: " <<endl <<t <<endl;
	cout <<endl;

    return 0;
}
