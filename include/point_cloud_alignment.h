#ifndef POINT_CLOUD_ALIGNMENT_H
#define POINT_CLOUD_ALIGNMENT_H

//#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Eigen>
#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Dense>
#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Eigen>

#include <opencv2/core/eigen.hpp>

void pointCloudAlign(Eigen::MatrixXf pointCloud_1, Eigen::MatrixXf pointCloud_2, cv::Mat &R, cv::Mat &t);
void pointCloudAlign_sac(Eigen::MatrixXf pointCloud_1, Eigen::MatrixXf pointCloud_2, cv::Mat &R, cv::Mat &t);

#endif