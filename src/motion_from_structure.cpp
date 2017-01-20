#include "motion_estimator.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <opencv2/core/core.hpp>     /* svd */

using namespace Eigen;
using namespace std;
using namespace cv;


void motionEstimator::motionFromStructure::updatePose(const std::vector<cv::Point3f> &point_cloud_1, const std::vector<cv::Point3f> &point_cloud_2, cv::Mat &R, cv::Mat &t)
{
    int num_point = point_cloud_1.size();
    int max_num_inlier = 0;
    vector<int> current_inlier_index, final_inlier_index;
    
    srand (time(NULL));
    vector<int> random_index;
    
    for (int i=0; i<max_iteration; i++){
        // Randomly pick three points from point cloud, each of them is different
        random_index.clear();
        random_index.push_back(rand() % num_point);
        random_index.push_back(rand() % num_point);
        while(random_index[0] == random_index[1])
            random_index[1] = rand() % num_point;
        random_index.push_back(rand() % num_point);
        while(random_index[0] == random_index[2] || random_index[1] == random_index[2])
            random_index[2] = rand() % num_point;
        
        // Compute pose use only random selected points
        pointCloudPose(point_cloud_1, point_cloud_2, random_index, R, t);
        
        // Based on computed model pose, select other inliers
        alsoInlierSet(point_cloud_1, point_cloud_2, R, t, current_inlier_index);
        
        if (current_inlier_index.size() > max_num_inlier){
            max_num_inlier = current_inlier_index.size();
            final_inlier_index = current_inlier_index;
        }
        
    }
    
    if (final_inlier_index.size() < 3)
        cerr<<"Not enough inliers to estimate pose!" <<endl;
    
    pointCloudPose(point_cloud_1, point_cloud_2, final_inlier_index, R, t);
    
}

void motionEstimator::motionFromStructure::pointCloudPose(const std::vector<cv::Point3f> &point_cloud_1, const std::vector<cv::Point3f> &point_cloud_2, std::vector<int> index, cv::Mat &R, cv::Mat &t)
{
    float p1_x=0, p1_y=0, p1_z=0;
    float p2_x=0, p2_y=0, p2_z=0;
    
    const int point_num = index.size();
    int p_index;
    for (int i=0; i<point_num; i++){
        p_index = index[i];
        p1_x += point_cloud_1[p_index].x;
        p1_y += point_cloud_1[p_index].y;
        p1_z += point_cloud_1[p_index].z;
        
        p2_x += point_cloud_2[p_index].x;
        p2_y += point_cloud_2[p_index].y;
        p2_z += point_cloud_2[p_index].z;
    }
    
    // centroids of two point clouds
    p1_x /= point_num;
    p1_y /= point_num;
    p1_z /= point_num;
    
    p2_x /= point_num;
    p2_y /= point_num;
    p2_z /= point_num;
    
    Mat H = Mat::zeros(3, 3, CV_32F);
    for (int i=0; i<point_num; i++){
        p_index = index[i];
        
        Point3f p1 = point_cloud_1[p_index] - Point3f(p1_x, p1_y, p1_z);
        Point3f p2 = point_cloud_2[p_index] - Point3f(p2_x, p2_y, p2_z);
        
        H.at<float>(0,0) += p1.x * p2.x;
        H.at<float>(0,1) += p1.x * p2.y;
        H.at<float>(0,2) += p1.x * p2.z;
        
        H.at<float>(1,0) += p1.y * p2.x;
        H.at<float>(1,1) += p1.y * p2.y;
        H.at<float>(1,2) += p1.y * p2.z;
        
        H.at<float>(2,0) += p1.z * p2.x;
        H.at<float>(2,1) += p1.z * p2.y;
        H.at<float>(2,2) += p1.z * p2.z;
        
    }
    
    Mat e, U, V, U_transpose;
    cv::SVDecomp(H, e, U, V, cv::SVD::FULL_UV);
    transpose(U, U_transpose);
    R = V * U_transpose;
    
//    cout <<p2_x <<" " <<p2_y <<" " <<p2_z <<endl;
//    cout <<R <<endl;
//    cout <<p1_x <<" " <<p1_y <<" " <<p1_z <<endl;
    t.at<float>(0) = p2_x - R.at<float>(0,0)*p1_x + R.at<float>(0,1)*p1_y + R.at<float>(0,2)*p1_z;
    t.at<float>(1) = p2_y - R.at<float>(1,0)*p1_x + R.at<float>(1,1)*p1_y + R.at<float>(1,2)*p1_z;
    t.at<float>(2) = p2_z - R.at<float>(2,0)*p1_x + R.at<float>(2,1)*p1_y + R.at<float>(2,2)*p1_z;
    
}
void motionEstimator::motionFromStructure::alsoInlierSet(const std::vector<cv::Point3f> &point_cloud_1, const std::vector<cv::Point3f> &point_cloud_2, const cv::Mat &R, const cv::Mat &t, std::vector<int> &inlier_index)
{
    if (inlier_index.size() != 0)
        inlier_index.clear();
    
    float residual = 0;
    Point3f p1, p2;
    float x, y, z;
    for (int i=0; i<point_cloud_1.size(); i++){
        p1 = point_cloud_1[i];
        p2 = point_cloud_2[i];
        
        x = p2.x - R.at<float>(0,0)*p1.x + R.at<float>(0,1)*p1.y + R.at<float>(0,2)*p1.z - t.at<float>(0);
        y = p2.y - R.at<float>(1,0)*p1.x + R.at<float>(1,1)*p1.y + R.at<float>(1,2)*p1.z - t.at<float>(1);
        z = p2.z - R.at<float>(2,0)*p1.x + R.at<float>(2,1)*p1.y + R.at<float>(2,2)*p1.z - t.at<float>(2);
        
        residual = x*x + y*y + z*z;
        
        if (residual <= tolerance)
            inlier_index.push_back(i);
    }
        
}

