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

using namespace std;
using namespace cv;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000


int test_monocular()
{
    Mat img1, img2;
    Mat R_res, t_res;
    
    // Code for test sequences
    char filename1[200], filename2[200];
    sprintf( filename1, "/Users/Muyuan/Downloads/dataset/sequences/00/image_0/%06d.png", 0);
    sprintf( filename2, "/Users/Muyuan/Downloads/dataset/sequences/00/image_0/%06d.png", 1);
    
    img1 = imread( filename1, CV_LOAD_IMAGE_GRAYSCALE);
    img2 = imread( filename2, CV_LOAD_IMAGE_GRAYSCALE);
    
    if(! img1.data || !img2.data )
    {
        cout <<" Could not open or find the image" << endl;
        return -1;
    }
    
    Eigen::MatrixXf P(3, 4);
    P << 718.8560,        0, 607.1928,         0,
    0, 718.8560, 185.2157,         0,
    0,        0,   1.0000,         0;
    
    /////////////////////////////////////////////////////////////////////
    // Code for test cal_pose(...)
    //int64 time = getTickCount();
    
    //cal_pose( img1_l, img1_r, img2_l, img2_r, P1, P2, R, t, method );
    
    //time = getTickCount() - time;
    //printf( "Time elapsed: %fms\n", time*1000/getTickFrequency() );
    /////////////////////////////////////////////////////////////////////
    vector<Point2f> features_prev, features_next;
    
    Mat R = Mat(3, 3, CV_32F, cvScalar(0.));
    Mat t = Mat(3, 1, CV_32F, cvScalar(0.));
    
    features_prev = monocular_pose(img1, img2, P, R, t);
    R_res = R.clone();
    t_res = t.clone();
    Mat previousImg = img2.clone();
    Mat currentImg;
    
    Mat traj = Mat::zeros(600, 600, CV_8UC3);
    
    char filename[200];
    
    showRes showTraj(traj);
    
    for (int iframe=2; iframe<MAX_FRAME; iframe++){
        sprintf(filename, "/Users/Muyuan/Downloads/dataset/sequences/00/image_0/%06d.png", iframe);
        currentImg = imread( filename, CV_LOAD_IMAGE_GRAYSCALE);
        
        monocular_pose( previousImg, currentImg, features_prev, features_next, P, R, t);
        
        double scale = getAbsoluteScale(iframe, 0, t.at<double>(2));
        
        if ( (scale>0.1) && t.at<double>(2) > t.at<double>(1) && t.at<double>(2) > t.at<double>(0) ){
            t_res = t_res + scale * (R_res * t);
            R_res = R * R_res;
        }
        
        if ( features_prev.size() < MIN_NUM_FEAT ){
            vector<uchar> status;
            featureDetection(previousImg, features_prev);
            featureTracking( previousImg, currentImg, features_prev, features_next, status);
        }
        
        // ===================================================
        // Display
        // ===================================================
        imshow("Camera", currentImg);
        showTraj.updateTraj(t_res);
        //int x = int(t_res.at<double>(0)) + 300;
        //int y = int(t_res.at<double>(2)) + 100;
        //
        //circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);
        //
        //rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
        //sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_res.at<double>(0), t_res.at<double>(1), t_res.at<double>(2));
        //putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
        //
        //imshow( "Trajectory", traj );
        //waitKey(1);
        // ===================================================
        
        previousImg = currentImg.clone();
        features_prev = features_next;
    }
    
    cout <<endl;
    cout <<endl;
    cout <<"Rotation Matrix: " <<endl <<R_res <<endl;
    cout <<endl;
    cout <<"Translation vector: " <<endl <<t_res <<endl;
    cout <<endl;
    
    return 0;
    
    
}


int main()
{
    test_monocular();
    
    return 0;
}



