#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "feature_matcher.h"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
void readme();


int main( int argc, char** argv )
{
//    clock_t begin = clock();
//    clock_t end;
//    double elapsed_secs;
//    
//    char f1_l[100], f1_r[100], f2[100];
//    sprintf(f1_l, "../data/img1_l.png");
//    sprintf(f1_r, "../data/img1_r.png");
//    sprintf(f2, "../data/img2_l.png");
//    
//    Mat img_1 = imread( f1_l, IMREAD_GRAYSCALE );
//    Mat img_2 = imread( f1_r, IMREAD_GRAYSCALE );
//    Mat img_3 = imread(   f2, IMREAD_GRAYSCALE );
//    if( !img_1.data || !img_2.data || !img_3.data )
//    { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
//    
//    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
//    Ptr<ORB> orb = ORB::create();
//    orb->setMaxFeatures(5000);
//    
//    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
//    Tracker orb_tracker(orb, matcher);
//    
//    orb_tracker.setFirstFrame(img_1 );
//    
//    end = clock();
//    elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
//    begin = end;
//    cout << " featureMatching :" << elapsed_secs << "s" <<endl;
//
//    orb_tracker.lrMatch(img_2);
//    
//    orb_tracker.pcMatch(img_3);
//    
//	
//    cout<< orb_tracker.first_kp.size() <<endl;
//    cout<< orb_tracker.first_right_kp.size() <<endl;
//    cout<< orb_tracker.current_kp.size() <<endl;
//    cout<< orb_tracker.good_matches.size() << endl;
//    //-- Draw only "good" matches
// 	Mat img_matches;
//    drawMatches( img_1, orb_tracker.first_kp, img_3, orb_tracker.current_kp,
//                orb_tracker.good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//    
//    
//    imshow( "Good Matches", img_matches );
//    
//    end = clock();
//    elapsed_secs = double( end - begin) / CLOCKS_PER_SEC;
//    cout << " featureMatching :" << elapsed_secs << "s" <<endl;
////
////    //-- Show detected matches
////    imshow( "Good Matches", img_matches );
////    for( int i = 0; i < (int)good_matches.size(); i++ )
////    { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }
//    waitKey(0);
//    return 0;
}
/*
 * @function readme
 */
void readme()
{ std::cout << " Usage: ./SURF_FlannMatcher <img1> <img2>" << std::endl; }
