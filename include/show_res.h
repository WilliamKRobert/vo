#ifndef SHOW_RES_H
#define SHOW_RES_H

#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/video/video.hpp>


using namespace std;
using namespace cv;


class showRes
{
public:
    int x, y;
    char text[100];
    int fontFace;
    double fontScale;
    int thickness;
    cv::Point textOrg;
    
    Mat plot;
	char *resFile;
	ofstream file;
    
	showRes( Mat &_plot, char *_resFile=NULL, double _fontFace=FONT_HERSHEY_PLAIN, double _fontScale=1, int _thickness=1, cv::Point _textOrg=Point(10, 50)): resFile(_resFile), plot(_plot), fontFace(_fontFace), fontScale(_fontScale), thickness(_thickness), textOrg(_textOrg) 
	{
		file.open(resFile);
		if (!file.is_open()){
			cerr<< "Could not open result file." <<endl;
		}
	}

	~showRes()
	{
		if (resFile != NULL)
			file.close();
	}
    
    void updateTraj(const Mat translation)
    {
		if ( translation.type() == CV_32F ){ 
	        x = int(translation.at<float>(0)) + 300;
        	y = int(translation.at<float>(2)) + 100;
		cout << translation.at<float>(0) <<endl;
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
    
};

#endif

