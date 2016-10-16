#include <fstream>
#include <cmath>

#include <opencv2/calib3d/calib3d.hpp>

#include "cal_pose.h"
#include "tool.h"

using namespace Eigen;
#include <opencv2/core/eigen.hpp>


vector<Point2f> monocular_pose(Mat img1, Mat img2, MatrixXf P, Mat &R, Mat &t)
{
    // bucket features
    int row = img1.rows, col = img1.cols;
    int h_break= 30, b_break= 40;
    int maxPerCell= 10;
    
    vector<KeyPoint> prevKeyPts;
    bucket_features(img1, prevKeyPts, row, col, h_break, b_break, maxPerCell);
    
    //Mat img_keypoints_1;
    //drawKeypoints( img1_l, prevKeyPts, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    
    //imshow("Keypoints in image 1", img_keypoints_1);
    
    // Comparison between bucketing and without bucketing
    //Ptr<FastFeatureDetector> fast1 = FastFeatureDetector::create();
    //vector<KeyPoint> kps;
    //Mat img_tmp;
    //fast1->detect(img1_l, kps);
    //drawKeypoints( img1_l, kps, img_tmp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    
    //imshow("Keypoints without bucketing", img_tmp);
    
    //waitKey(0);
    
    // optical flow tracking
    vector<uchar> status;
    vector<float> err;
    
    vector<Point2f> features_prev;
    vector<Point2f> features_next;
    
    //for (int i=0; i<prevKeyPts.size(); i++)
    //    features_prev.push_back(prevKeyPts[i].pt);
    featureDetection(img1, features_prev);
    
    Size winSize=Size(21, 21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    calcOpticalFlowPyrLK(img1, img2, features_prev, features_next, status, err, winSize, 3, termcrit, 0, 0.001);
    
    for (int i=0; i<status.size(); i++){
        if (status[i] == 0 || features_next[i].x<0 || features_next[i].y<0){
            features_prev.erase(features_prev.begin()+i);
            features_next.erase(features_next.begin()+i);
            status.erase(status.begin()+i);
            i--;
        }
    }
    
    // visualize the results of optical flow tracking
    //for (int i=0; i<status.size(); i++){
    //	line(img1_l, features_prev[i], features_next[i], Scalar(255,0,0));
    //	circle(img1_l, features_prev[i], 1, Scalar(0,0,0), -1);
    //}
    //imshow("Optical Flow", img1_l);
    //waitKey(0);
    
    double focal = P(0,0);
    Mat E, mask;
    Point2d pp(P(0,2), P(1, 2));

    E = findEssentialMat(features_next, features_prev, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, features_next, features_prev, R, t, focal, pp, mask);
    
    return features_next;
}

int monocular_pose(Mat img1, Mat img2, vector<Point2f> &features_prev, vector<Point2f> &features_next, MatrixXf P, Mat &R, Mat &t)
{
    // bucket features
    int row = img1.rows, col = img1.cols;
    int h_break= 30, b_break= 40;
    int maxPerCell= 10;
    
    vector<KeyPoint> prevKeyPts;
    bucket_features(img1, prevKeyPts, row, col, h_break, b_break, maxPerCell);
    
    //Mat img_keypoints_1;
    //drawKeypoints( img1_l, prevKeyPts, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    
    //imshow("Keypoints in image 1", img_keypoints_1);
    
    // Comparison between bucketing and without bucketing
    //Ptr<FastFeatureDetector> fast1 = FastFeatureDetector::create();
    //vector<KeyPoint> kps;
    //Mat img_tmp;
    //fast1->detect(img1_l, kps);
    //drawKeypoints( img1_l, kps, img_tmp, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    
    //imshow("Keypoints without bucketing", img_tmp);
    
    //waitKey(0);
    
    // optical flow tracking
    vector<uchar> status;
    vector<float> err;
    
    //for (int i=0; i<prevKeyPts.size(); i++)
    //    features_prev.push_back(prevKeyPts[i].pt);
    
    Size winSize=Size(21, 21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    calcOpticalFlowPyrLK(img1, img2, features_prev, features_next, status, err, winSize, 3, termcrit, 0, 0.001);
    
    for (int i=0; i<status.size(); i++){
        if (status[i] == 0 || features_next[i].x<0 || features_next[i].y<0){
            features_prev.erase(features_prev.begin()+i);
            features_next.erase(features_next.begin()+i);
            status.erase(status.begin()+i);
            i--;
        }
    }
    
    // visualize the results of optical flow tracking
    //for (int i=0; i<status.size(); i++){
    //	line(img1_l, features_prev[i], features_next[i], Scalar(255,0,0));
    //	circle(img1_l, features_prev[i], 1, Scalar(0,0,0), -1);
    //}
    //imshow("Optical Flow", img1_l);
    //waitKey(0);
    
    double focal = P(0,0);
    Mat E, mask;
    Point2d pp(P(0,2), P(1, 2));
    
    E = findEssentialMat(features_next, features_prev, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, features_next, features_prev, R, t, focal, pp, mask);
    
    return 0;
}

