/**
 * This file is part of vo.
 *
 * Muyuan Lin, 2016
 * For more information see <https://github.com/muyuanlin/vo>
 */

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <iostream>
#include "map.h"

class optimizer
{
public:
    optimizer(){}
    void localBundleAdjustment(const Map::MapAssociation &local_map, double* paramter,
                               const double fx, const double fy,
                               const double cx, const double cy);
    
        template <typename T>
        void Reprojection(const T* const camera,
                          const T* const point,
                          T &x, T &y)
        {
            T p[3];
            ceres::AngleAxisRotatePoint(camera, point, p);
            p[0] += camera[3];
            p[1] += camera[4];
            p[2] += camera[5];
    
            T xp = p[0] / p[2];
            T yp = p[1] / p[2];
    
//            T fx = T(707.0912);
//            T fy = T(707.0912);
//            T cx = T(601.8873);
//            T cy = T(183.1104);
            T fx = T(718.8560);
            T fy = T(718.8560);
            T cx = T(607.1928);
            T cy = T(185.2157);
            x = fx * xp + cx;
            y = fy * yp + cy;
        }
    
private:
    double fx, fy, cx, cy;
};
    // Templated pinhole camera model for used with Ceres.  The camera is
    // parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
    // focal length and 2 for radial distortion. The principal point is not modeled
    // (i.e. it is assumed be located at the image center).


struct SnavelyReprojectionError {
    SnavelyReprojectionError(double observed_x, double observed_y, double point_x, double point_y, double point_z)
    : observed_x(observed_x), observed_y(observed_y), point_x(point_x), point_y(point_y), point_z(point_z) {}
    
    template <typename T>
    bool operator()(const T* const camera,
                    T* residuals) const {
        // camera[0,1,2] are the angle-axis rotation.
        // camera[3,4,5] are the translation.
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        
        T p[3];
        T point[3];
        point[0] =  T(point_x);
        point[1] =  T(point_y);
        point[2] =  T(point_z);
        ceres::AngleAxisRotatePoint(camera, point, p);
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];
        
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];
        
//        T focal = T(718.8560);
//        T cx = T(607.1928);
//        T cy = T(185.2157);
//        T fx = T(707.0912);
//        T fy = T(707.0912);
//        T cx = T(601.8873);
//        T cy = T(183.1104);
        T fx = T(718.8560);
        T fy = T(718.8560);
        T cx = T(607.1928);
        T cy = T(185.2157);
        T predicted_x = fx * xp + cx;
        T predicted_y = fy * yp + cy;
        //Reprojection(camera, point, predicted_x, predicted_y);
        
        // The error is the difference between the predicted and observed position.
//        std::cout <<predicted_x <<"this  " <<predicted_y <<"and " <<observed_x <<"or " <<observed_y <<std::endl;
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y,
                                       const double point_x,
                                       const double point_y,
                                       const double point_z) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6>(
                                                                                   new SnavelyReprojectionError(observed_x, observed_y, point_x, point_y, point_z)));
    }
    
    double observed_x;
    double observed_y;
    double point_x, point_y, point_z;
};

//struct SnavelyReprojectionError {
//    SnavelyReprojectionError(double observed_x, double observed_y, double point_x, double point_y, double point_z)
//        : observed_x(observed_x), observed_y(observed_y), point_x(point_x), point_y(point_y), point_z(point_z) {}
//    
//    template <typename T>
//    bool operator()(const T* const camera,
//                    const T* const point_tmp,
//                    T* residuals) const {
//        // camera[0,1,2] are the angle-axis rotation.
//        T p[3];
//        
//        T point[3];
//        point[0] =  T(point_x);
//        point[1] =  T(point_y);
//        point[2] =  T(point_z);
//        
//        ceres::AngleAxisRotatePoint(camera, point, p);
//        
//        // camera[3,4,5] are the translation.
//        p[0] += camera[3];
//        p[1] += camera[4];
//        p[2] += camera[5];
//        
//        // Compute the center of distortion. The sign change comes from
//        // the camera model that Noah Snavely's Bundler assumes, whereby
//        // the camera coordinate system has a negative z axis.
//        T xp = p[0] / p[2];
//        T yp = p[1] / p[2];
//        
//        
//        T fx = T(707.0912);
//        T fy = T(707.0912);
//        T cx = T(601.8873);
//        T cy = T(183.1104);
//        T predicted_x = fx * xp + cx;
//        T predicted_y = fy * yp + cy;
//        
//        // The error is the difference between the predicted and observed position.
//        residuals[0] = predicted_x - T(observed_x);
//        residuals[1] = predicted_y - T(observed_y);
//        
//        return true;
//    }
//    
//    // Factory to hide the construction of the CostFunction object from
//    // the client code.
//    static ceres::CostFunction* Create(const double observed_x,
//                                       const double observed_y,
//                                       const double point_x,
//                                       const double point_y,
//                                       const double point_z){
//        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 3>(
//                                                                                   new SnavelyReprojectionError(observed_x, observed_y, point_x, point_y, point_z)));
//    }
//    
//    double observed_x;
//    double observed_y;
//    
//    double point_x, point_y, point_z;
//};


#endif
