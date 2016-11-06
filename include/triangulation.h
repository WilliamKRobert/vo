#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <opencv2/core/core.hpp>

//#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Eigen>
#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Dense>
#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Eigen>

Eigen::Vector4f triangulation(cv::Point2i ptInLeftImg, cv::Point2i ptInRightImg, Eigen::MatrixXf P1, Eigen::MatrixXf P2);

#endif



