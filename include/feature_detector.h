#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <iostream>

#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>

const int FAST_THRESHOLD = 100;

void featureDetection(cv::Mat img, std::vector<cv::Point2f>& points);

#endif

