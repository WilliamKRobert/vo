/**
 * This file is part of vo.
 *
 * Muyuan Lin, 2016
 * For more information see <https://github.com/muyuanlin/vo>
 */

#include "optimizer.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"



void optimizer::localBundleAdjustment(const localMapAssociation *local_map, double* paramter)
{
    // Use ceres to optimize local poses
    int num_cameras_  = local_map->num_cameras();
    int num_observations = local_map->num_observations();
    int num_points_ = local_map->num_points();
    
    std::cout <<"Standford SHIT again!: " <<num_observations <<std::endl;
    
    double observations[2*local_map->num_observations()];
    for (int i=0; i<num_observations; i++){
        observations[2*i]   = local_map->observations_[i].x;
        observations[2*i+1] = local_map->observations_[i].y;
    }
    
    for (int i=0; i<num_cameras_; i++){
        paramter[6*i] = local_map->camera_rotation_parameters_[i][0];
        paramter[6*i+1] = local_map->camera_rotation_parameters_[i][1];
        paramter[6*i+2] = local_map->camera_rotation_parameters_[i][2];
        paramter[6*i+3] = local_map->camera_translation_parameters_[i][0];
        paramter[6*i+4] = local_map->camera_translation_parameters_[i][1];
        paramter[6*i+5] = local_map->camera_translation_parameters_[i][2];
    }
        
    for (int i=0; i<num_points_; i++){
        paramter[6*num_cameras_ + 3*i] = local_map->point_parameters_[i].x;
        paramter[6*num_cameras_ + 3*i + 1] = local_map->point_parameters_[i].y;
        paramter[6*num_cameras_ + 3*i + 2] = local_map->point_parameters_[i].z;
    }

    
//    std::cout <<"Error here" <<std::endl;
//    
//    std::cout <<"Standford: " <<local_map.num_observations() <<std::endl;
//    std::cout <<"Hi: " <<local_map.num_points() <<std::endl;
//    double *temp = local_map.mutable_camera_for_observation(1);
//    double *ptemp = tmp->mutable_point_for_observation(6);//local_map.mutable_point_for_observation(6);
//    std::cout <<"This is a stupid code:" <<temp[0] <<std::endl;
//    std::cout <<"This is a stupid code:" <<temp[1] <<std::endl;
//    std::cout <<"This is a stupid code:" <<temp[2] <<std::endl;
//    std::cout <<"This is a stupid code:" <<temp[3] <<std::endl;
//    std::cout <<"This is a stupid code:" <<temp[4] <<std::endl;
//    std::cout <<"This is a stupid code:" <<temp[5] <<std::endl;
//    
//    std::cout <<"Ridiculous:----------------------------------" <<std::endl;
//    std::cout <<ptemp[0] <<" " <<ptemp[1] <<" " <<ptemp[2] <<std::endl;

    
    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;
    for (int i = 0; i < local_map->num_observations(); ++i) {
        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.
        ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(observations[2 * i + 0],
                                         observations[2 * i + 1]);
        problem.AddResidualBlock(cost_function,
                                 NULL /* squared loss */,
                                 &paramter[local_map->camera_index_[i]*6],
                                 &paramter[6*num_cameras_ + local_map->point_index_[i]*3]);
                                 //local_map.mutable_camera_for_observation(i),
                                 //local_map.mutable_point_for_observation(i));
        //std::cout <<a[0] <<" " <<a[1] <<" " <<a[2] <<std::endl;
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


