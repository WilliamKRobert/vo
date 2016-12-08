/**
 * This file is part of vo.
 *
 * Muyuan Lin, 2016
 * For more information see <https://github.com/muyuanlin/vo>
 */

#include "map_association.h"


using namespace std;
using namespace cv;


//void localMapAssociation::initValue(const localMapAssociation *init_local_map)
//{
//        num_cameras_ = init_local_map->num_cameras();
//        num_points_ = init_local_map->num_points();
//        num_parameters_ = 6 * num_cameras_ + 3 * num_points_;
//        num_observations_ = init_local_map->observations_.size();
//    
//        point_index_ = init_local_map->point_index_;
//        camera_index_ = init_local_map->camera_index_;
//    
//        observations_ = init_local_map->observations_;
//    
//        point_parameters_ = init_local_map->point_parameters_;
//        camera_rotation_parameters_ = init_local_map->camera_rotation_parameters_;
//        camera_translation_parameters_ = init_local_map->camera_translation_parameters_;
//    
//}
localMapAssociation::localMapAssociation(const localMapAssociation &init_local_map)
{
//    num_cameras_ = init_local_map.num_cameras();
//    num_points_ = init_local_map.num_points();
//    num_parameters_ = 6 * num_cameras_ + 3 * num_points_;
//    num_observations_ = init_local_map.observations_.size();
//    
//    point_index_ = init_local_map.point_index_;
//    camera_index_ = init_local_map.camera_index_;
//    
//    observations_ = init_local_map.observations_;
//    
//    point_parameters_ = init_local_map.point_parameters_;
//    camera_rotation_parameters_ = init_local_map.camera_rotation_parameters_;
//    camera_translation_parameters_ = init_local_map.camera_translation_parameters_;
//    
//    observations_array_ = new double(2*num_observations_);
//    //std::memcpy(observations_array_, init_local_map.observations_array_, 2*num_observations_);
//    for (int i=0; i<num_observations_; i++){
//        observations_array_[2*i] = init_local_map.observations_[i].x;
//        observations_array_[2*i+1] = init_local_map.observations_[i].y;
//    }
//    
//    parameters_ = new double(num_parameters_);
//    //cout <<"Num of parameters:" <<num_parameters_ <<endl;
//    //cout <<"Num:" <<sizeof(init_local_map.parameters_) / sizeof(*init_local_map.parameters_) <<endl;;
//    //std::memcpy(parameters_, init_local_map.parameters_, num_parameters_);
//    for (int i=0; i<num_cameras_; i++){
//        parameters_[6*i  ] = init_local_map.camera_rotation_parameters_[i].at<double>(0);
//        parameters_[6*i+1] = init_local_map.camera_rotation_parameters_[i].at<double>(1);
//        parameters_[6*i+2] = init_local_map.camera_rotation_parameters_[i].at<double>(2);
//        parameters_[6*i+3] = init_local_map.camera_translation_parameters_[i].at<double>(0);
//        parameters_[6*i+4] = init_local_map.camera_translation_parameters_[i].at<double>(1);
//        parameters_[6*i+5] = init_local_map.camera_translation_parameters_[i].at<double>(2);
//    }
//    
//    for (int i=0; i<num_points_; i++){
//        parameters_[6*num_cameras_ + 3*i  ] = init_local_map.point_parameters_[i].x;
//        parameters_[6*num_cameras_ + 3*i+1] = init_local_map.point_parameters_[i].y;
//        parameters_[6*num_cameras_ + 3*i+2] = init_local_map.point_parameters_[i].z;
//    }
//    
//    cout <<"---------------Final examination!----------------------" <<endl;
//    cout <<observations_array_[0] <<" " <<observations_array_[1] <<endl;
//    cout <<observations_array_[2] <<" " <<observations_array_[3] <<endl;
//    
//    cout <<"---------------Pose parameters!-----------------------" <<endl;
//    cout <<parameters_[0] <<" " <<parameters_[1] <<" " <<parameters_[2] <<" " <<parameters_[3] <<" " <<parameters_[4] <<" " <<parameters_[5] <<" " <<endl;
//    double *temp = this->mutable_camera_for_observation(1);
//    cout <<temp[0] <<" " <<temp[1] <<" " <<temp[2] <<" " <<temp[3] <<" " <<temp[4] <<" " <<temp[5] <<" " <<endl;
//    
//    cout <<"---------------Point parameters!-----------------------" <<endl;
////    double *ptemp = this->mutable_point_for_observation_readonly(6);
//    double *ptemp = &parameters_[6*num_cameras_+6*3];
//
//    cout <<ptemp[0] <<" " <<ptemp[1] <<" " <<ptemp[2] <<" " <<endl;
}


localMapAssociation::localMapAssociation(const vector<Point3f> point, const vector<Point2f> firstObservation, const Mat rvec, const Mat t)
{
    num_cameras_ = 1;
    num_points_ = point.size();
    num_parameters_ = num_cameras_ * 6 + num_points_ * 3;
    num_observations_ = firstObservation.size();
    
    point_parameters_ = point;
    
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

void localMapAssociation::addObservation(vector<Point2f> observed_points, vector<int> track_index, const Mat rvec, const Mat t){
    
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