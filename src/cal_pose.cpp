#include <iostream>

#include "cal_pose.h"

using namespace std;

int cal_pose(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r)
{
    // stereo map
    int numberOfDisparities = 112;
    int SADWindowSize = 9;
    float scale = 1.0;
    bool no_display = false;

    Mat disparityMap1, disparityMap2;
    stereo_matching(disparityMap1, img1_l, img1_r, STEREO_SGBM, numberOfDisparities, SADWindowSize, scale, no_display);
    stereo_matching(disparityMap2, img2_l, img2_r, STEREO_SGBM, numberOfDisparities, SADWindowSize, scale, no_display);

    // bucket features
    //int row = img1_l.rows;
    //int col = ing1_l.cols;

    //vector<KeyPoint> keypoints;
    //Fast(img1_l, keypoints, 9, true);
    //drawKeyPoints( img1_l, keypoints, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

    //imshow("Keypoints in image 1", img_keypoints_1);
    //waitKey(0);
    //
    //// optical flow tracking     
    //cv::calcOpticalFlowPyrLK(img1_l, img2_l, pts_prev, pts_next, status, err);
    return 0; 
}
