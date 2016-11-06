#ifndef FEATURE_TRACKING_H
#define FEATURE_TRACKING_H

#include <iostream>
#include <opencv2/core/core.hpp>


void featureTracking(cv::Mat img1_l, cv::Mat img1_r, cv::Mat img2_l, std::vector<cv::Point2f>& points1_l, std::vector<cv::Point2f>& points1_r, std::vector<cv::Point2f>& points2_l, std::vector<uchar>& status);

void featureTracking(cv::Mat img_1, cv::Mat img_2, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2, std::vector<uchar>& status);

void featureTracking(cv::Mat img1_l, cv::Mat img1_r, cv::Mat img2_l, cv::Mat img2_r, std::vector<cv::Point2f>& points1_l, std::vector<cv::Point2f>& points1_r, std::vector<cv::Point2f>& points2_l, std::vector<cv::Point2f> &points2_r);


#endif


