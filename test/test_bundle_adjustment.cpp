/*
 Test local bundle adjustment
 --------------------------------------------------------------------
 Muyuan Lin, 2016
 */

#include <cmath>
#include <cstdio>
#include <iostream>

#include <stdio.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>

#include "ceres/rotation.h"

#include "map.h"
#include "visualization.h"
#include "feature_detector.h"
#include "feature_tracker.h"
#include "triangulation.h"
#include "motion_estimator.h"
#include "optimizer.h"

using namespace std;
using namespace cv;

#define MAX_FRAME 1000//5000
#define MIN_NUM_FEATURES 100
#define CHAR_SIZE 200


int test_ba(char *dataset_dir, char *resFile, char *optimized_res_file)
{
    Mat R_res, t_res;
    
    double distance = 0;
    
    Mat traj = Mat::zeros(600, 600, CV_8UC3);
    Visualization showTraj(traj, resFile);
    Visualization optimized_res(optimized_res_file);
    
    // =======================================================
    // Read image file
    // =======================================================
    Mat img1_l, img1_r, img2_l, img2_r;
    char filename1_l[CHAR_SIZE], filename1_r[CHAR_SIZE], filename2_l[CHAR_SIZE], filename2_r[CHAR_SIZE];
    
    sprintf(filename1_l, "%simage_0/%06d.png", dataset_dir, 0);
    sprintf(filename1_r, "%simage_1/%06d.png", dataset_dir, 0 );
    sprintf(filename2_l, "%simage_0/%06d.png", dataset_dir, 1 );
    sprintf(filename2_r, "%simage_1/%06d.png", dataset_dir, 1);
    
   	img1_l = imread( filename1_l, CV_LOAD_IMAGE_GRAYSCALE );
   	img1_r = imread( filename1_r, CV_LOAD_IMAGE_GRAYSCALE );
   	img2_l = imread( filename2_l, CV_LOAD_IMAGE_GRAYSCALE );
   	img2_r = imread( filename2_r, CV_LOAD_IMAGE_GRAYSCALE );
    
    if(! img1_l.data || ! img1_r.data || !img2_l.data || !img2_r.data)
    {
        cout <<" Could not open or find the image" << endl;
        return 1;
    }
    
    /* 
     --------------------------------------------------------------------
     Read setting file
     Initialize camera parameter
     --------------------------------------------------------------------
     */
    Mat P1, P2, P;
    
    // KITTI 00-02
    string filename = "setting/KITTI_00-02.yml";
    FileStorage fs("../../../setting/KITTI_00-02.yml", FileStorage::READ);
    
    if (!fs.isOpened())
    {
        cerr << "failed to open " << filename << endl;
        return 1;
    }

    fs["camera_matrix"] >> P;
    fs["stereo_left_camera_matrix"] >> P1;
    fs["stereo_right_camera_matrix"] >> P2;
    
    int img_row, img_col;

    fs["camera_height"] >> img_row;
    fs["camera_width"] >>img_col;
    /*
     --------------------------------------------------------------------------------
     First pose estimation
     --------------------------------------------------------------------------------
     */
    Mat R = Mat::eye(3, 3, CV_64F);
    Mat t = Mat(3, 1, CV_64F, cvScalar(0.));
    showTraj.writeRes(R, t);
    optimized_res.writeRes(R, t);
    
    vector<Point2f> keypoints1_l, keypoints1_r, keypoints2_l, keypoints2_r;
    
    featureDetector detector(img_row, img_col, 100);
    detector.directDetect(img1_l, keypoints1_l);
   
    featureTracker tracker;
    tracker.featureTrack(img1_l, img1_r, img2_l, keypoints1_l, keypoints1_r, keypoints2_l);

    vector<int> init_index;
	tracker.getInitIndex(init_index);
    
    triangulation tri(P1, P2);
    vector<Point3f> point_cloud;
    tri.pc_triangulate(keypoints1_l, keypoints1_r, point_cloud);
    cout <<"How can it be?" <<point_cloud.size() <<endl;
    
    motionEstimator::motionFromStructureAndImage pose_estimator(P);
    pose_estimator.updatePose(point_cloud, keypoints2_l, R, t);
    
    R_res = R.clone();
    t_res = t.clone();
    showTraj.writeRes(R_res, t_res);
    
    Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    Rodrigues(R, rvec);
    
    Map::MapAssociation local_map(point_cloud, keypoints2_l, Mat::eye(3, 3, CV_64F), Mat(3, 1, CV_64F, cvScalar(0.)));
    
    Mat previousImg_l = img2_l.clone();
    Mat previousImg_r = img2_r.clone();
    Mat currentImg_l, currentImg_r;
    vector<Point2f> previousKeypoints = keypoints2_l;
    vector<Point2f> previousKeypoints_r;
    vector<Point2f> currentKeypoints;
    
    char filename_l[100], filename_r[100];
    google::InitGoogleLogging("Local bundle Adjustment");
    
    /*
     --------------------------------------------------------------------------------
     Incremental pose estimation
     --------------------------------------------------------------------------------
     */
   	int iframe;
    double *parameter = NULL;
    Mat key_frame_l = img1_l, key_frame_r = img1_r;
    vector<Point2f> key_frame_keypoints = keypoints1_l;
   	for (iframe=2; iframe<MAX_FRAME; iframe++){
        sprintf(filename_l, "%simage_0/%06d.png", dataset_dir, iframe);
        sprintf(filename_r, "%simage_1/%06d.png", dataset_dir, iframe);
        currentImg_l = imread( filename_l, CV_LOAD_IMAGE_GRAYSCALE);
        currentImg_r = imread( filename_r, CV_LOAD_IMAGE_GRAYSCALE);
        
        if(! currentImg_l.data)
        {
            cout << "\nTotal image pairs: " <<iframe <<endl;
            cout << "Finish image processing!" << endl;
            break;
        }
        
        tracker.featureTrack(previousImg_l, previousImg_r, currentImg_l, previousKeypoints, previousKeypoints_r, currentKeypoints);
        
        //tracker.featureTrack(key_frame_l, key_frame_r, currentImg_l, key_frame_keypoints, previousKeypoints_r, currentKeypoints);
        
        vector<int> track_index;
        tracker.getPointIndex(init_index, track_index);

        vector<Point3f> point_cloud;
        tri.pc_triangulate(previousKeypoints, previousKeypoints_r, point_cloud);
        
        pose_estimator.updatePose(point_cloud, currentKeypoints, R, t);
        
        if (abs(t.at<double>(2)) > abs(t.at<double>(1)) && abs(t.at<double>(2)) > abs(t.at<double>(0)) ){
            t_res = t_res + R_res * t;
            R_res = R * R_res;
//            distance  += sqrt( t.at<double>(0) * t.at<double>(0) + t.at<double>(2) * t.at<double>(2) );
            
        }
        
        Rodrigues(R_res, rvec);
		local_map.addObservation(currentKeypoints, track_index, rvec, t_res);
        
        init_index = track_index;


        if ( currentKeypoints.size() < MIN_NUM_FEATURES ){
            
            
            
            
            int num_cameras = local_map.num_cameras();
            int num_points = local_map.num_points();
            int num_parameters = 6*num_cameras + 3*num_points;
            cout <<"-----------------" <<num_cameras <<"-----------------" <<endl;
            if (parameter != NULL) {
                delete[] parameter;
                parameter = NULL;
            }
            
            parameter = new double[num_parameters];
            
            //local_optimizer = optimizer();
            optimizer local_optimizer;
            local_optimizer.localBundleAdjustment(local_map, parameter, P1.at<double>(0,0), P1.at<double>(1,1), P1.at<double>(0,2), P1.at<double>(1,2));
        
            imshow("Camera", currentImg_l);
            showTraj.updateTraj(t_res);
            showTraj.writeRes(R_res, t_res);
            
            for(int j=0; j<local_map.num_cameras(); j++){
                optimized_res.writeRes(&parameter[6*j]);
                
            }
            
            int nn = local_map.num_cameras()-1;
            if (abs(parameter[6*nn+3]-t_res.at<double>(0)) < 2 &&
                abs(parameter[6*nn+4]-t_res.at<double>(1)) < 2 &&
                abs(parameter[6*nn+5]-t_res.at<double>(2)) < 2 ){
                t_res.at<double>(0) = parameter[6*nn+3];
                t_res.at<double>(1) = parameter[6*nn+4];
                t_res.at<double>(2) = parameter[6*nn+5];
                
                rvec.at<double>(0) = parameter[6*nn+0];
                rvec.at<double>(1) = parameter[6*nn+1];
                rvec.at<double>(2) = parameter[6*nn+2];
                
                Rodrigues(rvec, R_res);
                
            }
            
            vector<uchar> status;
            detector.directDetect(currentImg_l, currentKeypoints);
            
            previousImg_l = currentImg_l.clone();
            previousImg_r = currentImg_r.clone();
            previousKeypoints = currentKeypoints;
            
            iframe++;
            sprintf(filename_l, "%simage_0/%06d.png", dataset_dir, iframe);
            sprintf(filename_r, "%simage_1/%06d.png", dataset_dir, iframe);
            currentImg_l = imread( filename_l, CV_LOAD_IMAGE_GRAYSCALE);
            currentImg_r = imread( filename_r, CV_LOAD_IMAGE_GRAYSCALE);
            
            if(! currentImg_l.data)
            {
                cout << "\nTotal image pairs: " <<iframe <<endl;
                cout << "Finish image processing!" << endl;
                break;
            }
            
            tracker.featureTrack(previousImg_l, previousImg_r, currentImg_l, previousKeypoints, previousKeypoints_r, currentKeypoints);
            
//            vector<int> track_index;
//            tracker.getPointIndex(init_index, track_index);
            tracker.getInitIndex(init_index);
            
            vector<Point3f> point_cloud;
            tri.pc_triangulate(previousKeypoints, previousKeypoints_r, point_cloud);
            
            pose_estimator.updatePose(point_cloud, currentKeypoints, R, t);
            
            if (abs(t.at<double>(2)) > abs(t.at<double>(1)) && abs(t.at<double>(2)) > abs(t.at<double>(0)) ){
                t_res = t_res + R_res * t;
                R_res = R * R_res;
                distance  += sqrt( t.at<double>(0) * t.at<double>(0) + t.at<double>(2) * t.at<double>(2) );
                
            }
            
            local_map = Map::MapAssociation(point_cloud, currentKeypoints, R_res, t_res);
        }
        
        imshow("Camera", currentImg_l);
        showTraj.updateTraj(t_res);
        showTraj.writeRes(R_res, t_res);
        
        previousImg_l = currentImg_l.clone();
        previousImg_r = currentImg_r.clone();
        previousKeypoints = currentKeypoints;
    }
    
    
    waitKey(0);
    return 0;
    
}


int main(int argc, char *argv[])
{
    if ( argc != 2 ){
        cerr << "Usage: ./test_stereo test_sequence_no" << endl;
        return -1;
    }
    
    char dataset_dir[CHAR_SIZE] = "/Users/Muyuan/Documents/vo/evaluation/kitti/data/sequences/";
    
    char res_file[CHAR_SIZE];
    char optimized_res_file[CHAR_SIZE];
    int seq_no = atoi(argv[1]);
    
    cout <<"Enjoy XCode!" <<endl;
    sprintf(dataset_dir, "%s%.02d/", dataset_dir, seq_no);
    sprintf(res_file, "/Users/Muyuan/vo/evaluation/results/data/%.02d.txt", seq_no);
    sprintf(optimized_res_file, "/Users/Muyuan/vo/evaluation/results/data/optimized_%.02d.txt", seq_no);
    
    test_ba(dataset_dir, res_file, optimized_res_file);
    
    return 0;
}
