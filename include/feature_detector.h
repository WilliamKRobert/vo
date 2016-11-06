#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <iostream>

#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>

int bucket_features(cv::Mat I, std::vector<cv::KeyPoint> &keypoints, int h, int b, int h_break, int b_break, int numCorners);

void featureDetection(cv::Mat img, std::vector<cv::Point2f>& points, int threshold = 100);

#endif

