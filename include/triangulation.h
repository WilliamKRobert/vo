#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <opencv2/core/core.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigen>



class triangulation
{
public:
    triangulation(Eigen::MatrixXf camera_matrix_1_, Eigen::MatrixXf camera_matrix_2_): camera_matrix_1(camera_matrix_1_), camera_matrix_2(camera_matrix_2_){}
    
    Eigen::Vector4f triangulate(cv::Point2i ptInLeftImg, cv::Point2i ptInRightImg);
    
    void pc_triangulate(std::vector<cv::Point2f> &keypoionts_1, std::vector<cv::Point2f> &keypoionts_2, std::vector<cv::Point3f> &point_cloud);
    
private:
    Eigen::MatrixXf camera_matrix_1, camera_matrix_2;
};
#endif



