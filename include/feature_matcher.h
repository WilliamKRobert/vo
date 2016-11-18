#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H

#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <iostream>
#include <opencv2/core/core.hpp>

using namespace cv;

void bruteForceMatching(cv::Mat leftImageGrey, cv::Mat rightImageGrey, std::vector<cv::Point2f> &k1, std::vector<cv::Point2f> &k2);

int stereo_sparse_matching(cv::Mat img1_l, cv::Mat img1_r, cv::Mat img2_l, cv::Mat img2_r, std::vector<cv::Point2f> &keypoints1_l, std::vector<cv::Point2f> &keypoints1_r, std::vector<cv::Point2f> &keypoints2_l, std::vector<cv::Point2f> &keypoints2_r);

const double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio

class Matcher
{
public:
    Matcher(Ptr<Feature2D> _detector, Ptr<DescriptorMatcher> _matcher) :
    detector(_detector),
    matcher(_matcher)
    {}
    
    void initFirstFrame(const Mat frame);
    void setFirstFrame(const Mat frame);
    void findMatch(const Mat frame);
    void getMatchKeypoint(std::vector<KeyPoint> &kpt1, std::vector<KeyPoint> &kpt2)
    {
        kpt1 = first_match_kpt;
        kpt2 = second_match_kpt;
    }
    Ptr<Feature2D> getDetector() {
        return detector;
    }
    
    Ptr<Feature2D> detector;
    Ptr<DescriptorMatcher> matcher;
    
    std::vector< DMatch > good_matches;

    Mat first_frame, first_desc, first_match_desc;
    std::vector<KeyPoint> first_kpt, first_match_kpt;
    
    Mat second_frame, second_desc, second_match_desc;
    std::vector<KeyPoint> second_kpt, second_match_kpt;
};

#endif

