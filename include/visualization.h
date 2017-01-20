#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/video/video.hpp>

#include "ceres/rotation.h"


using namespace std;
using namespace cv;


class Visualization
{
public:
    Mat plot;
    char *resFile;
    
    int x, y;
    char text[100];
    int fontFace;
    double fontScale;
    int thickness;
    cv::Point textOrg;
    
	ofstream file;
    
	Visualization( Mat &_plot, char *_resFile=NULL, double _fontFace=FONT_HERSHEY_PLAIN, double _fontScale=1, int _thickness=1, cv::Point _textOrg=Point(10, 50)): plot(_plot), resFile(_resFile), fontFace(_fontFace), fontScale(_fontScale), thickness(_thickness), textOrg(_textOrg)
	{
		file.open(resFile);
		if (!file.is_open()){
			cerr<< "Could not open result file." <<endl;
		}
	}
    
    Visualization( char *_resFile=NULL): resFile(_resFile)
    {
        file.open(resFile);
        if (!file.is_open()){
            cerr<< "Could not open optimized result file." <<endl;
        }
    }

	~Visualization()
	{
		if (resFile != NULL)
			file.close();
	}
    
    void updateTraj(const Mat translation)
    {
		if ( translation.type() == CV_32F ){ 
	        x = int(translation.at<float>(0)) + 300;
        	y = int(translation.at<float>(2)) + 100;
		}
		else if( translation.type() == CV_64F ){
			x = int(translation.at<double>(0)) + 300;
        	y = int(translation.at<double>(2)) + 100;
		} 
		else{
			cout <<"translation vector type is not correct!" <<endl;
			return;
		}
        circle(plot, Point(x, y) ,1, CV_RGB(255,0,0), 2);
        
        rectangle( plot, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);

		if ( translation.type() == CV_32F ){
        	sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", translation.at<float>(0), translation.at<float>(1), translation.at<float>(2));
		}
		else if( translation.type() == CV_64F ){
			sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));

		}
		else{
			return;
		}
			
        putText( plot, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8 );
        
        imshow( "Trajectory", plot );
        waitKey(1);
    }

    void writeRes(const Mat rotation, const Mat translation)
    {
        if ( translation.type() == CV_32F ){
            file << rotation.at<float>(0,0) << " " <<
            rotation.at<float>(0,1) << " " <<
            rotation.at<float>(0,2) << " " <<
            translation.at<float>(0) << " ";
            file << rotation.at<float>(1,0) << " " <<
            rotation.at<float>(1,1) << " " <<
            rotation.at<float>(1,2) << " " <<
            translation.at<float>(1) << " ";
            file << rotation.at<float>(2,0) << " " <<
            rotation.at<float>(2,1) << " " <<
            rotation.at<float>(2,2) << " " <<
            translation.at<float>(2) << " ";
            file << "\n";
        }
        else if( translation.type() == CV_64F ){
            file << rotation.at<double>(0,0) << " " <<
            rotation.at<double>(0,1) << " " <<
            rotation.at<double>(0,2) << " " <<
            translation.at<double>(0) << " ";
            file << rotation.at<double>(1,0) << " " <<
            rotation.at<double>(1,1) << " " <<
            rotation.at<double>(1,2) << " " <<
            translation.at<double>(1) << " ";
            file << rotation.at<double>(2,0) << " " <<
            rotation.at<double>(2,1) << " " <<
            rotation.at<double>(2,2) << " " << 
            translation.at<double>(2) << " ";
            file << "\n";
            
        }
        else 
            cerr << "Unknown result data type." <<endl;
    }

    
	void writeRes(const double *const pose_array)
	{
        double rotation_matrix[9];
        double rotation_array[3] = {pose_array[0], pose_array[1], pose_array[2]};
        ceres::AngleAxisToRotationMatrix(rotation_array, rotation_matrix);
        
        file << rotation_matrix[0] <<" ";
        file << rotation_matrix[3] <<" ";
        file << rotation_matrix[6] <<" ";
        file << pose_array[3]        <<" ";
        
        file << rotation_matrix[1] <<" ";
        file << rotation_matrix[4] <<" ";
        file << rotation_matrix[7] <<" ";
        file << pose_array[4]        <<" ";
        
        file << rotation_matrix[2] <<" ";
        file << rotation_matrix[5] <<" ";
        file << rotation_matrix[8] <<" ";
        file << pose_array[5]             ;
        
        file << "\n";
	}
    
};

#endif

