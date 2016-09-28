#include "cal_pose.h" 
using namespace Eigen;


int cal_pose(Mat img1_l, Mat img1_r, Mat img2_l, Mat img2_r, MatrixXf P1, MatrixXf P2)
{
    // stereo map
    int numberOfDisparities = 112;
    int SADWindowSize = 9;
    float scale = 1.0;
    bool no_display = false;

    Mat disparityMap1, disparityMap2;
    stereo_matching(disparityMap1, img1_l, img1_r, STEREO_SGBM, numberOfDisparities, SADWindowSize, scale, no_display);
    stereo_matching(disparityMap2, img2_l, img2_r, STEREO_SGBM, numberOfDisparities, SADWindowSize, scale, no_display);

    // bucket features
    int row = img1_l.rows, col = img1_l.cols;
    int h_break= 50, b_break= 80;
    int maxPerCell= 5;

    vector<KeyPoint> prevKeyPts;
    bucket_features(img1_l, prevKeyPts, row, col, h_break, b_break, maxPerCell);

    Mat img_keypoints_1;
    drawKeypoints( img1_l, prevKeyPts, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    
    imshow("Keypoints in image 1", img_keypoints_1);
    waitKey(0);
    
    // optical flow tracking     
    vector<uchar> status;
    vector<float> err;

    vector<Point2f> features_prev;
    vector<Point2f> features_next; 

    for (int i=0; i<prevKeyPts.size(); i++)
        features_prev.push_back(prevKeyPts[i].pt);

    calcOpticalFlowPyrLK(img1_l, img2_l, features_prev, features_next, status, err);

    // triangulation of 3D point cloud
    for (int i=0; i<status.size(); i++){
        if (status[i] == 0){
            features_prev.erase(features_prev.begin()+i);
            features_next.erase(features_next.begin()+i);
            status.erase(status.begin()+i);
            i--; 
        }
    }

    vector<Point2i> M1_l(features_prev.begin(), features_prev.end());
    vector<Point2i> M2_l(features_next.begin(), features_next.end());
    vector<Point2i> M1_r(M1_l);
    vector<Point2i> M2_r(M2_l);
    
    vector<int> allow(features_prev.size(), 0);

    cout <<M1_l.size() <<endl;
    for (int i=0; i<M1_l.size(); i++){
        if ( (disparityMap1.at<int>(M1_l[i].y, M1_l[i].x) >0) && (disparityMap1.at<int>(M1_l[i].y, M1_l[i].x) < 100) && (disparityMap2.at<int>(M2_l[i].y, M2_l[i].x) > 0) && (disparityMap2.at<int>(M2_l[i].y, M2_l[i].x) < 100) )
        {
            M1_r[i].x = ( M1_l[i].x - disparityMap1.at<int>(M1_l[i].y, M1_l[i].x));
            M2_r[i].x = ( M2_l[i].x - disparityMap2.at<int>(M2_l[i].y, M2_l[i].x));
            allow[i] = 1;

        }
        else{
            M1_l.erase(M1_l.begin()+i);
            M1_r.erase(M1_r.begin()+i);
            M2_l.erase(M2_l.begin()+i);
            M2_r.erase(M2_r.begin()+i);
            i--;
        }
    }    

    MatrixXf point3D_1(M1_l.size(), 4);
    MatrixXf point3D_2(M2_l.size(), 4);

    Matrix4f A;
    
    cout <<M1_l.size() <<endl;;
    for (int i=0; i<M1_l.size(); i++){
        A.block<1,4>(0,0) = M1_l[i].x * P1.block<1,4>(2,0) - P1.block<1, 4>(0,0); 
        A.block<1,4>(1,0) = M1_l[i].y * P1.block<1,4>(2,0) - P1.block<1, 4>(1,0); 
        A.block<1,4>(2,0) = M1_r[i].x * P2.block<1,4>(2,0) - P2.block<1, 4>(0,0); 
        A.block<1,4>(3,0) = M1_r[i].y * P2.block<1,4>(2,0) - P2.block<1, 4>(1,0); 

        JacobiSVD<MatrixXf> svdOfA(A, ComputeThinV);
        const Eigen::MatrixXf V = svdOfA.matrixV(); 
        
        Vector4f X = V.block<4,1>(0, 3);
        X = X/X(3);

        point3D_1.block<1, 4>(i, 0) = X;
        cout <<V.rows() <<" " <<V.cols() <<endl;;
    }
    // the inlier detection

    // computation of R and t

    return 0; 
}
