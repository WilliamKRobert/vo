#include "feature_detector.h"

using namespace std;
using namespace cv;


void featureDetection(Mat img_1, vector<Point2f>& points1)	{
    //uses FAST as of now, modify parameters as necessary
    vector<KeyPoint> keypoints_1;
    int fast_threshold = FAST_THRESHOLD;
    bool nonmaxSuppression = true;
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    KeyPoint::convert(keypoints_1, points1, vector<int>());
}


