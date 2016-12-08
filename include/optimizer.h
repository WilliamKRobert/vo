/**
 * This file is part of vo.
 *
 * Muyuan Lin, 2016
 * For more information see <https://github.com/muyuanlin/vo>
 */

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <iostream>
#include "map_association.h"

class optimizer
{
public:
    optimizer(){}
    void localBundleAdjustment(const localMapAssociation * init_local_map, localMapAssociation &local_map);
    void localBundleAdjustment(const localMapAssociation *local_map, double* paramter);
private:
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
        
        T focal = T(718.8560);
        T cx = T(607.1928);
        T cy = T(185.2157);
        x = focal * xp + cx;
        y = focal * yp + cy;
    }

    
    // Templated pinhole camera model for used with Ceres.  The camera is
    // parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
    // focal length and 2 for radial distortion. The principal point is not modeled
    // (i.e. it is assumed be located at the image center).
};

struct SnavelyReprojectionError {
    SnavelyReprojectionError(double observed_x, double observed_y)
    : observed_x(observed_x), observed_y(observed_y) {}
    
    template <typename T>
    bool operator()(const T* const camera,
                    const T* const point,
                    T* residuals) const {
        // camera[0,1,2] are the angle-axis rotation.
        // camera[3,4,5] are the translation.
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];
        
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];
        
        T focal = T(718.8560);
        T cx = T(607.1928);
        T cy = T(185.2157);
        T predicted_x = focal * xp + cx;
        T predicted_y = focal * yp + cy;
        //Reprojection(camera, point, predicted_x, predicted_y);
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 3>(
                                                                                   new SnavelyReprojectionError(observed_x, observed_y)));
    }
    
    double observed_x;
    double observed_y;
};

#endif
