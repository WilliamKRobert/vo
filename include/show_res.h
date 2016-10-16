#ifndef SHOW_RES_H
#define SHOW_RES_H

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/video/video.hpp>


using namespace std;


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
    
    showRes( Mat &_plot, double _fontFace=FONT_HERSHEY_PLAIN, double _fontScale=1, int _thickness=1, cv::Point _textOrg=Point(10, 50)): plot(_plot), fontFace(_fontFace), fontScale(_fontScale), thickness(_thickness), textOrg(_textOrg) {}
    
    void updateTraj(Mat translation)
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
    
};

#endif

