#include <iostream>

#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Dense>
#include </usr/local/Cellar/eigen/3.2.8/include/eigen3/Eigen/Eigen>

#include "tool.h"
#include "triangulation.h"

using namespace std;
using namespace cv;


int main( int argc, char** argv )
{

	Eigen::MatrixXf P1(3, 4), P2(3, 4);
    P1 << 718.8560,        0, 607.1928,         0,
    	 		 0, 718.8560, 185.2157,         0,
			     0,        0,   1.0000,         0;
    P2 << 718.8560,        0, 607.1928, -386.1448,
                 0, 718.8560, 185.2157,         0,
                 0,        0,   1.0000,         0;
	
	Point2i p1(10, 5), p2(8, 5);
    triangulation tri(P1, P2);
	Eigen::Vector4f a = tri.triangulate(p1, p2);
	cout << a << endl;

	return 0;

}

