#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>

#include "motion_estimator.h"

using namespace Eigen;
using namespace std;
using namespace cv;
#include <opencv2/core/eigen.hpp>


void motionEstimator::motionFromImage::updatePose(vector<Point2f> &features_prev, vector<Point2f> &features_next,Mat &R, Mat &t)
{
    Mat E, mask;
    
    E = findEssentialMat(features_next, features_prev, focal, offset, method, prob, threshold, mask);
    recoverPose(E, features_next, features_prev, R, t, focal, offset, mask);
    
    return;
}

