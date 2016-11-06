#ifndef CAL_POSE_H
#define CAL_POSE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/video/video.hpp>

//#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Eigen>
#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Dense>
#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Eigen>
#include <iostream>
#include <vector>

using namespace cv;
using namespace cv::xfeatures2d;

/*------------------------------------------------------
 * odometry paramters
 * ----------------------------------------------------*/
const char DATASET_PATH[] = "/Users/Muyuan/Downloads/dataset/sequences/00/";

enum Alg{ STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };

void stereo_pose(Mat img1_l, Mat img1_r, Mat img2_l, std::vector<Point2f> &keypoints1_l, std::vector<Point2f> &keypoints2_l, Eigen::MatrixXf P1, Eigen::MatrixXf P2, Mat P, Mat &R, Mat &t);

/*
void stereo_pose(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r, Eigen::MatrixXf &pointCloud_1, Eigen::MatrixXf &pointCloud_2, Eigen::MatrixXf P1, Eigen::MatrixXf P2, Mat &R, Mat &t);
void stereo_pose(Mat img_l, Mat img_r, Eigen::MatrixXf &pointCloud_1, Eigen::MatrixXf &pointCloud_2, Eigen::MatrixXf P1, Eigen::MatrixXf P2, Mat &R, Mat &t);
*/
std::vector<Point2f> monocular_pose(Mat img1, Mat img2, Eigen::MatrixXf P, Mat &R, Mat &t);
int monocular_pose(Mat img1, Mat img2, std::vector<Point2f> &features_prev, std::vector<Point2f> &features_next, Eigen::MatrixXf P, Mat &R, Mat &t);

void featureMatching(Mat leftImageGrey, Mat rightImageGrey, std::vector<Point2f> &k1, std::vector<Point2f> &k2);

int disparity_map(Mat &disp8, Mat img1, Mat img2, Alg alg, int numberOfDisparities, int SADWindowSize, float scale, bool no_dispaly);
int stereo_sparse_matching(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r, std::vector<Point2f> &keypoints1_l, std::vector<Point2f> &keypoints1_r, std::vector<Point2f> &keypoints2_l, std::vector<Point2f> &keypoints2_r);

int bucket_features(Mat I, std::vector<KeyPoint> &keypoints, int h, int b, int h_break, int b_break, int numCorners);

Eigen::Vector4f triangulation(Point2i ptInLeftImg, Point2i ptInRightImg, Eigen::MatrixXf P1, Eigen::MatrixXf P2);

#endif



