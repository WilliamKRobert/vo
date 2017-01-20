#ifndef MOTION_ESTIMATOR_H
#define MOTION_ESTIMATOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <vector>

using namespace cv;
using namespace cv::xfeatures2d;

/*------------------------------------------------------
 * odometry paramters
 * ----------------------------------------------------*/
const char DATASET_PATH[] = "/Users/Muyuan/Documents/vo/evaluation/kitti/data/sequences/00/";

void stereo_pose(Mat img1_l, Mat img1_r, Mat img2_l, std::vector<Point2f> &keypoints1_l, std::vector<Point2f> &keypoints2_l, Eigen::MatrixXf P1, Eigen::MatrixXf P2, Mat P, Mat &R, Mat &t);

/*
void stereo_pose(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r, Eigen::MatrixXf &pointCloud_1, Eigen::MatrixXf &pointCloud_2, Eigen::MatrixXf P1, Eigen::MatrixXf P2, Mat &R, Mat &t);
void stereo_pose(Mat img_l, Mat img_r, Eigen::MatrixXf &pointCloud_1, Eigen::MatrixXf &pointCloud_2, Eigen::MatrixXf P1, Eigen::MatrixXf P2, Mat &R, Mat &t);
*/

//std::vector<Point2f> monocular_pose(Mat img1, Mat img2, Eigen::MatrixXf P, Mat &R, Mat &t);
//int monocular_pose(Mat img1, Mat img2, std::vector<Point2f> &features_prev, std::vector<Point2f> &features_next, Eigen::MatrixXf P, Mat &R, Mat &t);

class motionEstimator
{
public:
    // 2D to 2D pose estimation
    class motionFromImage
    {
    public:
        motionFromImage(double focal_, Point2d offset_, int method_=cv::RANSAC, double prob_=0.999, double threshold_=1.0):focal(focal_), offset(offset_), method(method_), prob(prob_), threshold(threshold_){}
        
        void updatePose(std::vector<Point2f> &features_prev, std::vector<Point2f> &features_next, Mat &R, Mat &t);
        
    private:
        // camera parameter
        double focal;
        cv::Point2d offset;
        
        // parameter for RANSAC or MEDS
        int method;
        double prob;
        double threshold;
    };
    
    // ICP problem
    class motionFromStructure
    {
    public:
        motionFromStructure(int max_iteration_ = 2000000, double tolerance_ = .01):max_iteration(max_iteration_), tolerance(tolerance_) {}
        
        void updatePose(const std::vector<cv::Point3f> &point_cloud_1, const std::vector<cv::Point3f> &point_cloud_2, cv::Mat &R, cv::Mat &t);
        
    private:
        int max_iteration;
        double tolerance;
        
        bool useExtrinsicGuess;
        int iterationsCount;
        float reprojectionError;
        float confidence;
        
        void pointCloudPose(const std::vector<cv::Point3f> &point_cloud_1, const std::vector<cv::Point3f> &point_cloud_2, std::vector<int> random_index, cv::Mat &R, cv::Mat &t);
        void alsoInlierSet(const std::vector<cv::Point3f> &point_cloud_1, const std::vector<cv::Point3f> &point_cloud_2, const cv::Mat &R, const cv::Mat &t, std::vector<int> &inlier_index);
    };
    
    // PnP problem
    class motionFromStructureAndImage
    {
    public:
        motionFromStructureAndImage(cv::Mat intrinsic_matrix_, bool useExtrinsicGuess_ = false, int iterationsCount_ = 1000, float reprojectionError_ = 1.0, float confidence_ = 0.95): intrinsic_matrix(intrinsic_matrix_), useExtrinsicGuess(useExtrinsicGuess_), iterationsCount(iterationsCount_), reprojectionError(reprojectionError_), confidence(confidence_)
        {}
        
        void updatePose(const std::vector<cv::Point3f> &point_cloud, const std::vector<cv::Point2f> &points, cv::Mat &R, cv::Mat &t);
        
    private:
        cv::Mat intrinsic_matrix;
        
        bool useExtrinsicGuess;
        int iterationsCount;            // number of Ransac iterations.
        float reprojectionError;        // maximum allowed distance to consider it an inlier,
                                        //      very important to the final accuracy, the best parameter
                                        //      is 1.0 for FAST, threshold 3, 0.8 for SIFT.
        float confidence;               // ransac successful confidence.
    };
};

#endif



