#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>

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

using namespace std;
using namespace cv;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000


int test_stereo()
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
    
    Eigen::MatrixXf P1(3, 4), P2(3, 4);
    P1 << 718.8560,        0, 607.1928,         0,
    0, 718.8560, 185.2157,         0,
    0,        0,   1.0000,         0;
    P2 << 718.8560,        0, 607.1928, -386.1448,
    0, 718.8560, 185.2157,         0,
    0,        0,   1.0000,         0;
    
    // =======================================================
    // Initialization
    // =======================================================
    
    //vector<Point2f> features_prev, features_next;

    Mat R = Mat(3, 3, CV_32F, cvScalar(0.));
    Mat t = Mat(3, 1, CV_32F, cvScalar(0.));
    
    vector<Point2f> keypoints1_l, keypoints1_r, keypoints2_l, keypoints2_r;
    featureDetection(img1_l, keypoints1_l);
    stereo_pose(img1_l, img1_r, img2_l, keypoints1_l, keypoints2_l, P1, P2, R, t);
    
    R_res = R.clone();
    t_res = t.clone();
    Mat previousImg_l = img2_l.clone();
    Mat previousImg_r = img2_r.clone();
    Mat currentImg_l, currentImg_r;
    vector<Point2f> previousKeypoints = keypoints2_l;
    vector<Point2f> currentKeypoints;
    
    Mat traj = Mat::zeros(600, 600, CV_8UC3);
    
    char filename_l[100], filename_r[100];
    
    showRes showTraj(traj);
    
    // =======================================================
    // Loop
    // =======================================================
    
   	for (int iframe=2; iframe<MAX_FRAME; iframe++){
        sprintf(filename_l, "/Users/Muyuan/Downloads/dataset/sequences/00/image_0/%06d.png", iframe);
        sprintf(filename_r, "/Users/Muyuan/Downloads/dataset/sequences/00/image_1/%06d.png", iframe);
        currentImg_l = imread( filename_l, CV_LOAD_IMAGE_GRAYSCALE);
        currentImg_r = imread( filename_r, CV_LOAD_IMAGE_GRAYSCALE);
        
        stereo_pose(previousImg_l, previousImg_r, currentImg_l, previousKeypoints, currentKeypoints, P1, P2, R, t);
        
//        cout <<R << " " <<endl <<t <<endl;
//        cout <<R_res << " "<<endl << t_res << endl;
        if (abs(t.at<double>(2)) > abs(t.at<double>(1)) && abs(t.at<double>(2)) > abs(t.at<double>(0)) ){
            t_res = t_res + R_res * (-t);
            R_res = R * R_res;

        }
        
        if ( currentKeypoints.size() < MIN_NUM_FEAT ){
        	vector<uchar> status;
        	featureDetection(currentImg_l, currentKeypoints);
        }
        
        imshow("Camera", currentImg_l);
        showTraj.updateTraj(t_res);
        
        previousImg_l = currentImg_l.clone();
        previousImg_r = currentImg_r.clone();
        previousKeypoints = currentKeypoints;
    }
    
    waitKey(0);
    return 0;
    
}


int main()
{
    test_stereo();
    
    return 0;
}

