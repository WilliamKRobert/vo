#include "cal_pose.h"
#include <vector>

using std::vector;

vector<int> linspace(int a, int b, int n)
{
    vector<int> array;
    int step = (b-1) / (n-1);
    
    while( a<= b){
        array.push_back(a);
        a += step;
    }

    return array;
}

int bucket_features(Mat I, vector<KeyPoint> &keypoints, int h, int b, int h_break, int b_break, int numCorners)
{
// Instead of dividing the image I into several blocks and find features 
//      on each block, here we randomly pick some features from detection 
//      results of whole image I.
    int threshold = 9;
    
    FAST(I, keypoints, threshold, true);
      
    return 0;
}
