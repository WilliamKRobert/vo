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
using namespace std;

enum Alg{ STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };

int cal_pose(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r, Eigen::MatrixXf P1, Eigen::MatrixXf P2, Mat &R, Mat &t, int method);

int stereo_matching(Mat &disp8, Mat img1, Mat img2, Alg alg, int numberOfDisparities, int SADWindowSize, float scale, bool no_dispaly);

int bucket_features(Mat I, vector<KeyPoint> &keypoints, int h, int b, int h_break, int b_break, int numCorners);

Eigen::Vector4f triangulation(Point2i ptInLeftImg, Point2i ptInRightImg, Eigen::MatrixXf P1, Eigen::MatrixXf P2);

void removeRow(Eigen::MatrixXf& matrix, unsigned int rowToRemove);

void removeColumn(Eigen::MatrixXf& matrix, unsigned int colToRemove);

/*
template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, cv::Mat& dst);

template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void cv2eigen( const Mat& src,
		               Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& dst );

template<typename _Tp>
void cv2eigen( const Mat& src,
		               Eigen::Matrix<_Tp, Eigen::Dynamic, Eigen::Dynamic>& dst );

template<typename _Tp>
void cv2eigen( const Mat& src,
		               Eigen::Matrix<_Tp, Eigen::Dynamic, 1>& dst );

template<typename _Tp>
void cv2eigen( const Mat& src,
		               Eigen::Matrix<_Tp, 1, Eigen::Dynamic>& dst );
*/
#endif



