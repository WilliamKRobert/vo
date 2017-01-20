/**
 * This file is part of vo.
 *
 * Muyuan Lin, 2016
 * For more information see <https://github.com/muyuanlin/vo>
 */

#include "optimizer.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"



void optimizer::localBundleAdjustment(const Map::MapAssociation &local_map, double* paramter,
                                      const double fx_, const double fy_,
                                      const double cx_, const double cy_)
{
    // camera parameter
    fx = fx_;
    fy = fy_;
    cx = cx_;
    cy = cy_;
    
    // Use ceres to optimize local poses
    int num_cameras_  = local_map.num_cameras();
    int num_observations = local_map.num_observations();
    int num_points_ = local_map.num_points();
    
    double observations[2*local_map.num_observations()];
    for (int i=0; i<num_observations; i++){
        observations[2*i]   = local_map.observations_[i].x;
        observations[2*i+1] = local_map.observations_[i].y;
    }
    
    for (int i=0; i<num_cameras_; i++){
        paramter[6*i] = local_map.camera_rotation_parameters_[i][0];
        paramter[6*i+1] = local_map.camera_rotation_parameters_[i][1];
        paramter[6*i+2] = local_map.camera_rotation_parameters_[i][2];
        paramter[6*i+3] = local_map.camera_translation_parameters_[i][0];
        paramter[6*i+4] = local_map.camera_translation_parameters_[i][1];
        paramter[6*i+5] = local_map.camera_translation_parameters_[i][2];
    }
        
    for (int i=0; i<num_points_; i++){
        paramter[6*num_cameras_ + 3*i    ] = local_map.point_parameters_[i].x;
        paramter[6*num_cameras_ + 3*i + 1] = local_map.point_parameters_[i].y;
        paramter[6*num_cameras_ + 3*i + 2] = local_map.point_parameters_[i].z;
    }
    
    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;
    
    int camera_tag = local_map.camera_index_[0];
    int track_index = 0;
    for (int i = 0; i < local_map.num_observations(); ++i) {
        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.
        if (track_index > 100){
            if (camera_tag == local_map.camera_index_[i]){
                continue;
            }
            else{
                camera_tag = local_map.camera_index_[i];
                track_index = 0;
                continue;
            }
        }
        else{
            double x, y;
            Reprojection(&paramter[local_map.camera_index_[i]*6], &paramter[num_cameras_*6 + local_map.point_index_[i]*3],
                         x, y);
            x = observations[2 * i + 0] - x;
            y = observations[2 * i + 1] - y;
//            std::cout <<x <<" " <<y <<" " << observations[2 * i + 0] <<" " <<observations[2 * i + 1] <<std::endl;
            if ( x*x + y*y < 300){
            
            double *point_i = &paramter[6*num_cameras_ + local_map.point_index_[i]*3];
            ceres::CostFunction* cost_function =
            SnavelyReprojectionError::Create(observations[2 * i + 0],
                                             observations[2 * i + 1],
                                             point_i[0],
                                             point_i[1],
                                             point_i[2]);
            problem.AddResidualBlock(cost_function,
                                     NULL /* squared loss */,
                                     &paramter[local_map.camera_index_[i]*6]//,
                                     //&paramter[num_cameras_*6 + local_map.point_index_[i]*3]
                                     );
            
            track_index++;
            }
        }
        
        
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    // If solve small to medium sized problems, consider setting
    // use_explicit_schur_complement as true
    // options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}


