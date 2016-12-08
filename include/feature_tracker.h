#ifndef FEATURE_TRACKING_H
#define FEATURE_TRACKING_H

#include <iostream>
#include <opencv2/core/core.hpp>


class featureTracker
{
public:
	featureTracker();
	~featureTracker(){}
	
    // void getStatus(vector<uchar>& status_){status_ = status;}
    
    // only between two images
    void featureTrack(cv::Mat img_1, cv::Mat img_2, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2);
    
    // between left and right, fist left and second left images
    void featureTrack(cv::Mat img1_l, cv::Mat img1_r, cv::Mat img2_l, std::vector<cv::Point2f>& points1_l, std::vector<cv::Point2f>& points1_r, std::vector<cv::Point2f>& points2_l);
    
    // between left and right, second left and second right images
    void featureTrack(cv::Mat img1_l, cv::Mat img1_r, cv::Mat img2_l, cv::Mat img2_r, std::vector<cv::Point2f>& points1_l, std::vector<cv::Point2f>& points1_r, std::vector<cv::Point2f>& points2_l, std::vector<cv::Point2f> &points2_r);

    void getInitIndex(std::vector<int> &init_index);
	void getPointIndex(std::vector<int> &init_index, std::vector<int> &new_index);


private:
	std::vector<float> err;
    cv::Size winSize;
    cv::TermCriteria termcrit;
    std::vector<uchar> status_1, status_2;
	
    int feature_num;
};
#endif


