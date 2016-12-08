#include <iostream>
#include <stdio.h>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/eigen.hpp"

#include "tool.h"
#include "show_res.h"
#include "feature_detector.h"
#include "feature_tracker.h"
#include "motion_estimator.h"


using namespace std;
using namespace cv;

#define MAX_FRAME 1000 //4540 
#define MIN_NUM_FEATURES 2000
const int FAST_THRESHOLD = 3;


int test_monocular()
{
    Mat img1, img2;
    Mat R_res, t_res;
    
	clock_t begin = clock();
	clock_t end;
	double elapsed_secs;

    char filename1[200], filename2[200];
    sprintf( filename1, "%simage_0/%06d.png", DATASET_PATH, 0);
    sprintf( filename2, "%simage_0/%06d.png", DATASET_PATH, 1);
    
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
    
	int img_row = img1.rows, img_col = img1.cols;

    vector<Point2f> features_prev, features_next;

	featureDetector detector(img_row, img_col, 100);
	//detector.bucketingDetect(img1, features_prev);
	detector.directDetect(img1, features_prev);
    cout <<features_prev.size() <<endl;	
	featureTracker tracker;
	tracker.featureTrack(img1, img2, features_prev, features_next);

    Mat R = Mat(3, 3, CV_32F, cvScalar(0.));
    Mat t = Mat(3, 1, CV_32F, cvScalar(0.));
    
	double focal = P(0,0);
	Point2d offset(P(0,2), P(1,2));
	motionEstimator::motionFromImage pose_estimator(P(0,0), offset);
	pose_estimator.updatePose(features_prev, features_next, R, t);	
   	cout <<R <<endl <<t <<endl; 
    R_res = R.clone();
    t_res = t.clone();
    Mat previousImg = img2.clone();
	features_prev = features_next;

	Mat currentImg;
    Mat traj = Mat::zeros(600, 600, CV_8UC3);
    char filename[200];
   	double scale; 

    showRes showTraj(traj);
    detector.setThreshold(3); 
    for (int iframe=2; iframe<MAX_FRAME; iframe++){
        sprintf(filename, "%simage_0/%06d.png", DATASET_PATH, iframe);
        currentImg = imread( filename, CV_LOAD_IMAGE_GRAYSCALE );
		tracker.featureTrack(previousImg, currentImg, features_prev, features_next);

		pose_estimator.updatePose(features_prev, features_next, R, t);
        scale = getAbsoluteScale(iframe, 0, t.at<double>(2));
        
        if ( (scale>0.1) && t.at<double>(2) > t.at<double>(1) && t.at<double>(2) > t.at<double>(0) ){
            t_res = t_res + scale * (R_res * t);
            R_res = R * R_res;
        }
        
        if ( features_prev.size() < MIN_NUM_FEATURES ){
			//detector.bucketingDetect(previousImg, features_prev);
			detector.directDetect(previousImg, features_prev);

			tracker.featureTrack(previousImg, currentImg, features_prev, features_next);
        }
        
        imshow("Camera", currentImg);
        showTraj.updateTraj(t_res);
        
        previousImg = currentImg.clone();
        features_prev = features_next;

    }
    
	end = clock();
	elapsed_secs = double( end - begin ) / CLOCKS_PER_SEC;
	cout <<"Total duration for " <<MAX_FRAME << " frames: " <<elapsed_secs <<" s" <<endl;
	cout << "Frame rate: " <<MAX_FRAME / elapsed_secs <<" fps" <<endl;

    cout <<endl <<endl;
    cout <<"Rotation Matrix: " <<endl <<R_res <<endl;
    cout <<endl;
    cout <<"Translation vector: " <<endl <<t_res <<endl;
    cout <<endl;

   	waitKey(0);

    return 0;
}


int main()
{
    test_monocular();
    
    return 0;
}



