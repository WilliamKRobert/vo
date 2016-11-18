#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>

class featureDetector
{
public:
    // no bucketing using FAST
    featureDetector(){}
    // bucketing using FAST
    featureDetector(int img_row_, int img_col_, int threshold_ = 10, int row_break_ = 30, int col_break_ = 40, int max_per_cell_ = 10):
        img_row(img_row_), img_col(img_col_), threshold(threshold_), row_break(row_break_), col_break(col_break_),max_per_cell(max_per_cell_){}
    
    void bucketingDetect(cv::Mat img, std::vector<cv::Point2f> &points);

    void directDetect(cv::Mat img, std::vector<cv::Point2f>& points);
    
    void setThreshold(int threshold_){threshold = threshold_;}

private:
    int img_row, img_col;
    int row_break, col_break;
    int max_per_cell;
    
    int threshold;
    
    struct ResponseComparator;
    void keepStrongest( int N, std::vector<cv::KeyPoint>& keypoints );
    
};

#endif

