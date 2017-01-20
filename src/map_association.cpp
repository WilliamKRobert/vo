/**
 * This file is part of vo.
 *
 * Muyuan Lin, 2016
 * For more information see <https://github.com/muyuanlin/vo>
 */

#include "map.h"
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

Map::MapAssociation::MapAssociation(const vector<Point3f> point, const vector<Point2f> firstObservation, const Mat rmat, const Mat t)
{
    Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    Rodrigues(rmat, rvec);
    
    Mat rmat_inv = Mat::eye(3, 3, CV_64F);
    rmat_inv = rmat.inv();
    vector<Point3f> point_0;
    float x, y, z;
    for (int i=0; i<point.size(); i++){
        x =  rmat_inv.at<double>(0,0) * (point[i].x - t.at<double>(0))
           + rmat_inv.at<double>(0,1) * (point[i].y - t.at<double>(1))
           + rmat_inv.at<double>(0,2) * (point[i].z - t.at<double>(2));
        y =  rmat_inv.at<double>(1,0) * (point[i].x - t.at<double>(0))
           + rmat_inv.at<double>(1,1) * (point[i].y - t.at<double>(1))
           + rmat_inv.at<double>(1,2) * (point[i].z - t.at<double>(2));
        z =  rmat_inv.at<double>(2,0) * (point[i].x - t.at<double>(0))
           + rmat_inv.at<double>(2,1) * (point[i].y - t.at<double>(1))
           + rmat_inv.at<double>(2,2) * (point[i].z - t.at<double>(2));
        
        Point3f tmp(x, y, z);
        point_0.push_back(tmp);
    }
    
    
    num_cameras_ = 1;
    num_points_ = point.size();
    num_parameters_ = num_cameras_ * 6 + num_points_ * 3;
    num_observations_ = firstObservation.size();
    
    point_parameters_ = point_0;
    
    for (int i=0; i<firstObservation.size(); i++){
        camera_index_.push_back(0);
        point_index_.push_back(i);
        observations_.push_back(firstObservation[i]);
        
    }
    
    std::vector<double> rvec_copy;
    rvec_copy.push_back(rvec.at<double>(0));
    rvec_copy.push_back(rvec.at<double>(1));
    rvec_copy.push_back(rvec.at<double>(2));
    std::vector<double> t_copy;
    t_copy.push_back(t.at<double>(0));
    t_copy.push_back(t.at<double>(1));
    t_copy.push_back(t.at<double>(2));
    
    camera_rotation_parameters_.push_back(rvec_copy);
    camera_translation_parameters_.push_back(t_copy);
    
}

void Map::MapAssociation::addObservation(vector<Point2f> observed_points, vector<int> track_index, const Mat rvec, const Mat t){
    
    num_cameras_++;
    num_parameters_ += 6;
    num_observations_ += observed_points.size();
    
    for (int i=0; i<observed_points.size(); i++){
        camera_index_.push_back(num_cameras_-1);
        point_index_.push_back(track_index[i]);
        observations_.push_back(observed_points[i]);
    }
    
    std::vector<double> rvec_copy;
    rvec_copy.push_back(rvec.at<double>(0));
    rvec_copy.push_back(rvec.at<double>(1));
    rvec_copy.push_back(rvec.at<double>(2));
    std::vector<double> t_copy;
    t_copy.push_back(t.at<double>(0));
    t_copy.push_back(t.at<double>(1));
    t_copy.push_back(t.at<double>(2));
    
    camera_rotation_parameters_.push_back(rvec_copy);
    camera_translation_parameters_.push_back(t_copy);
}

//void Map::MapAssociation::
