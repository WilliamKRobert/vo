#ifndef DISPARITY_MAP_H
#define DISPARITY_MAP_H

#include <opencv2/core/core.hpp>

enum Alg{ STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };

int disparity_map(cv::Mat &disp8, cv::Mat img1, cv::Mat img2, Alg alg, int numberOfDisparities, int SADWindowSize, float scale, bool no_dispaly);

#endif



