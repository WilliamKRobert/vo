#include "feature_detector.h"


#include <iostream>
#include <opencv2/core/core.hpp>

//#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Eigen>
#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Dense>
#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Eigen>

using namespace std;
using namespace cv;
using namespace Eigen;


/*************************************************************
 * bucketing feature
 *************************************************************
*/
struct ResponseComparator
{
    bool operator() (const KeyPoint& a, const KeyPoint& b)
    {
        return std::abs(a.response) > std::abs(b.response);
    }
};

static void keepStrongest( int N, vector<KeyPoint>& keypoints )
{
    if( (int)keypoints.size() > N )
    {
        vector<KeyPoint>::iterator nth = keypoints.begin() + N;
        std::nth_element( keypoints.begin(), nth, keypoints.end(), ResponseComparator() );
        keypoints.erase( nth, keypoints.end() );
    }
}

int bucket_features(Mat I, vector<KeyPoint> &keypoints, int h, int b, int h_break, int b_break, int maxPerCell)
{
    VectorXd y = VectorXd::LinSpaced(h_break, 0, h-h_break);
    VectorXd x = VectorXd::LinSpaced(b_break, 0, b-b_break);
    
    Ptr<FastFeatureDetector> fast = FastFeatureDetector::create();
    
    for (int i=0; i<y.size(); i++){
        for (int j=0; j<x.size(); j++){
            Mat subImage = I(Range(y[i], y[i]+h_break), Range(x[j], x[j]+b_break));
            vector<KeyPoint> sub_keypoints;
            sub_keypoints.reserve(maxPerCell);
            
            fast->detect(subImage, sub_keypoints);
            keepStrongest(maxPerCell, sub_keypoints);
            
            vector<KeyPoint>::iterator it = sub_keypoints.begin(), end = sub_keypoints.end();
            for( ; it != end; ++it){
                it->pt.x += x[j];
                it->pt.y += y[i];
            }
            
            keypoints.insert(keypoints.end(), sub_keypoints.begin(), sub_keypoints.end());
        }
    }
    
    
    return 0;
}

/*************************************************************
 * feature detection
 *************************************************************
 */
void featureDetection(Mat img_1, vector<Point2f>& points1, int threshold)	{
    //uses FAST as of now, modify parameters as necessary
    vector<KeyPoint> keypoints_1;
    int fast_threshold = threshold;
    bool nonmaxSuppression = true;
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    KeyPoint::convert(keypoints_1, points1, vector<int>());
}


