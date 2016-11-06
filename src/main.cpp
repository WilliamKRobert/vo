#include <iostream>
#include <stdio.h>
#include <fstream>

#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Dense>
#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/eigen.hpp"

#include "cal_pose.h"
#include "tool.h"
#include "show_res.h"
#include "feature_detector.h"

using namespace std;
using namespace cv;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

int main()
{
	Mat img1_l, img1_r, img2_l, img2_r;
    Mat R_res, t_res;
    
	// =======================================================
	// Preprocessing
	// =======================================================
    char filename1_l[200], filename1_r[200], filename2_l[200], filename2_r[200];
    sprintf( filename1_l, "/Users/Muyuan/Downloads/dataset/sequences/00/image_0/%06d.png", 0);
    sprintf( filename1_r, "/Users/Muyuan/Downloads/dataset/sequences/00/image_1/%06d.png", 0);
    sprintf( filename2_l, "/Users/Muyuan/Downloads/dataset/sequences/00/image_0/%06d.png", 1);
    sprintf( filename2_r, "/Users/Muyuan/Downloads/dataset/sequences/00/image_1/%06d.png", 1);
    
   	img1_l = imread( filename1_l, CV_LOAD_IMAGE_GRAYSCALE );
   	img1_r = imread( filename1_r, CV_LOAD_IMAGE_GRAYSCALE );
   	img2_l = imread( filename2_l, CV_LOAD_IMAGE_GRAYSCALE );
   	img2_r = imread( filename2_r, CV_LOAD_IMAGE_GRAYSCALE );
    
    if(! img1_l.data || ! img1_r.data || !img2_l.data || !img2_r.data)
    {
    	cout <<" Could not open or find the image" << endl;
        return -1;
    }

	vector<Point2f> f1_l, f1_r, f2_l, f2_r;
    featureDetection(img1_l, f1_l);
    
    Mat R, t;
    
    Eigen::MatrixXf P1(3, 4), P2(3, 4);
    P1 << 718.8560,        0, 607.1928,         0,
    0, 718.8560, 185.2157,         0,
    0,        0,   1.0000,         0;
    P2 << 718.8560,        0, 607.1928, -386.1448,
    0, 718.8560, 185.2157,         0,
    0,        0,   1.0000,         0;
    
	Mat P = (Mat_<double>(3, 3) <<718.8560, 0, 607.1928, 0, 718.8560, 185.2157, 0, 0, 1);

    //stereo_pose(img1_l, img1_r, img2_l, f1_l, f2_l, P1, P2, P, R, t);
	vector<uchar> status;
	featureDetection(img1_l, f1_l);
	featureTracking(img1_l, img1_r, f1_l, f1_r, status);
	
    // visualize the results of optical flow tracking
    for (int i=0; i<f1_l.size(); i++){
    	line(img1_l, f1_l[i], f1_r[i], Scalar(255,0,0));
    	circle(img1_l, f1_l[i], 1, Scalar(0,0,0), -1);
		//line(img1_l, f1_l[i], f2_l[i], Scalar(255,0,0));
    	//circle(img1_l, f1_l[i], 1, Scalar(0,0,0), -1);

    }
    imshow("Optical Flow 1", img1_l);

    waitKey(0);
	return 0;
}

