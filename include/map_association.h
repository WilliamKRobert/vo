/**
 * This file is part of vo.
 *
 * Muyuan Lin, 2016
 * For more information see <https://github.com/muyuanlin/vo>
 */

#ifndef MAP_ASSOCIATION_H
#define MAP_ASSOCIATION_H

#include <cmath>
#include <cstdio>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include "ceres/ceres.h"
#include "ceres/rotation.h"


class localMapAssociation {
public:
    localMapAssociation(){}
    localMapAssociation(const localMapAssociation &init_local_map);
    localMapAssociation(const std::vector<cv::Point3f> point, const std::vector<cv::Point2f> firstObservation, const cv::Mat rvec, const cv::Mat t);
    
//    void initValue(const localMapAssociation *init_local_map);

    
//    ~localMapAssociation() {
//        
////        std::cout <<"Got this bitch!" <<std::endl;
////        
////        delete[] observations_array_;
////        delete[] parameters_;
////
////        observations_array_ = NULL;
////        parameters_ = NULL;
////        std::cout <<"this jerk!" <<std::endl;
//    }
    
    int num_observations()       const { return num_observations_;               }
//    const double* observations() const { return observations_array_;             }
    
    int num_cameras()            const{  return num_cameras_;                    }
    int num_points()             const{  return num_points_;                     }
    
    
    
//    double* mutable_cameras()          { return parameters_;                     }
//    double* mutable_points()           { return parameters_  + 6 * num_cameras_; }
    
//    double* mutable_camera_for_observation_readonly(int i) const{
//        return mutable_cameras_readonly() + camera_index_[i] * 6;
//    }
//    double* mutable_cameras_readonly()    const  { return parameters_;                     }
    
//    double* mutable_point_for_observation_readonly(int i)  const{
//        return mutable_points_readonly() + point_index_[i] * 3;
//    }
    
//    double* mutable_points_readonly()      const     { return mutable_cameras_readonly()  + 6 * num_cameras_; }

    
    
    
//    double* mutable_camera_for_observation(int i) {
//        return mutable_cameras() + camera_index_[i] * 6;
//    }
//    double* mutable_point_for_observation(int i) {
//        return mutable_points() + point_index_[i] * 3;
//    }

    void addObservation(std::vector<cv::Point2f> observed_points, std::vector<int> track_index, const cv::Mat rvec, const cv::Mat t);
    
    std::vector<cv::Point2f> observations_;
    std::vector<std::vector<double>> camera_rotation_parameters_;
    std::vector<std::vector<double>> camera_translation_parameters_;
    std::vector<cv::Point3f> point_parameters_;
    
    std::vector<int> point_index_;
    std::vector<int> camera_index_;

private:
    int num_cameras_;
    int num_points_;
    int num_parameters_;
    int num_observations_;
    
//    double *observations_array_;
//    double *parameters_;
};
#endif
