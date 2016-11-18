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
struct featureDetector::ResponseComparator
{
    bool operator() (const KeyPoint& a, const KeyPoint& b)
    {
        return std::abs(a.response) > std::abs(b.response);
    }
};

void featureDetector::keepStrongest( int N, vector<KeyPoint>& keypoints )
{
    if( (int)keypoints.size() > N )
    {
        vector<KeyPoint>::iterator nth = keypoints.begin() + N;
        std::nth_element( keypoints.begin(), nth, keypoints.end(), ResponseComparator() );
        keypoints.erase( nth, keypoints.end() );
    }
}

void featureDetector::bucketingDetect(Mat I, vector<Point2f> &points)
{
    vector<KeyPoint> keypoints;
    
    VectorXd y = VectorXd::LinSpaced(row_break, 0, img_row-row_break);
    VectorXd x = VectorXd::LinSpaced(col_break, 0, img_col-col_break);
    
    Ptr<FastFeatureDetector> fast = FastFeatureDetector::create(threshold);
    
    for (int i=0; i<y.size(); i++){
        for (int j=0; j<x.size(); j++){
            Mat subImage = I(Range(y[i], y[i]+row_break), Range(x[j], x[j]+col_break));
            vector<KeyPoint> sub_keypoints;
            sub_keypoints.reserve(max_per_cell);
            
            fast->detect(subImage, sub_keypoints);
            keepStrongest(max_per_cell, sub_keypoints);
            
            vector<KeyPoint>::iterator it = sub_keypoints.begin(), end = sub_keypoints.end();
            for( ; it != end; ++it){
                it->pt.x += x[j];
                it->pt.y += y[i];
            }
            
            keypoints.insert(keypoints.end(), sub_keypoints.begin(), sub_keypoints.end());
        }
    }
    
    KeyPoint::convert(keypoints, points, vector<int>());

    
}

/*************************************************************
 * feature detection without bucketing
 *************************************************************
 */
void featureDetector::directDetect(Mat img, vector<Point2f>& points)	{
    //uses FAST as of now, modify parameters as necessary
    vector<KeyPoint> keypoints;
    bool nonmaxSuppression = true;
    
    FAST(img, keypoints, threshold, nonmaxSuppression);
    KeyPoint::convert(keypoints, points, vector<int>());
}
