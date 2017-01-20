/**
 * This file is part of vo.
 *
 * Muyuan Lin, 2016
 * For more information see <https://github.com/muyuanlin/vo>
 */

#ifndef MAP_H
#define MAP_H

#include <cmath>
#include <cstdio>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include "ceres/ceres.h"
#include "ceres/rotation.h"


class Map {
public:
    class MapAssociation{
        
    public:
        MapAssociation(const std::vector<cv::Point3f> point, const std::vector<cv::Point2f> firstObservation, const cv::Mat rvec, const cv::Mat t);

        
        int num_observations()       const { return num_observations_;               }
        int num_cameras()            const{  return num_cameras_;                    }
        int num_points()             const{  return num_points_;                     }
        
        
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
    };
};
#endif
