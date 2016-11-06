#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H

#include <iostream>
#include <opencv2/core/core.hpp>

void featureMatching(cv::Mat leftImageGrey, cv::Mat rightImageGrey, std::vector<cv::Point2f> &k1, std::vector<cv::Point2f> &k2);

int stereo_sparse_matching(cv::Mat img1_l, cv::Mat img1_r, cv::Mat img2_l, cv::Mat img2_r, std::vector<cv::Point2f> &keypoints1_l, std::vector<cv::Point2f> &keypoints1_r, std::vector<cv::Point2f> &keypoints2_l, std::vector<cv::Point2f> &keypoints2_r);

#endif

