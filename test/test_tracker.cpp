#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "tool.h"
#include "feature_detector.h"
#include "feature_tracker.h"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


int main( int argc, char** argv )
{
    clock_t begin = clock();
    clock_t end;
    double elapsed_secs;
    
    featureTracker optical_flow_tracker;
    optical_flow_tracker.initTracker();
    
    if( argc != 3 )
    { cout <<"Wrong parameters!" <<endl; return -1; }
    Mat img_1 = imread( argv[1], IMREAD_GRAYSCALE );
    Mat img_2 = imread( argv[2], IMREAD_GRAYSCALE );
    if( !img_1.data || !img_2.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
    
    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    vector<Point2f> points1, points2;
    vector<uchar> status;
    
    featureDetector detector;
    detector.directDetect(img_1, points1);
    
    optical_flow_tracker.featureTrack(img_1, img_2, points1, points2);
    
    end = clock();
    elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
    cout << " feature tracking :" << elapsed_secs << "s" <<endl;
    
    // visualize the results of optical flow tracking
    for (int i=0; i<points1.size(); i++){
    	line(img_1, points1[i], points2[i], Scalar(255,0,0));
    	circle(img_1, points1[i], 1, Scalar(0,0,0), -1);
    }
    imshow("Optical Flow 1", img_1);
    
    waitKey(0);

    
    return 0;
}