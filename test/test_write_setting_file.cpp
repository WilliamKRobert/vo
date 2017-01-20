#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;

int main(int, char** argv)
{
	FileStorage fs("KITTI_04-12.yml", FileStorage::WRITE);

	if (!fs.isOpened()){
		std::cerr << "failed to open setting file" <<std::endl;
		return 1;
	}

	fs << "camera_fx" << 707.0912;
	fs << "camera_fy" << 707.0912;
	fs << "camera_cx" << 601.8873;
	fs << "camera_cy" << 183.1104;

	//fs << "\n";

	fs << "camera_k1" << 0.0;
	fs << "camera_k2" << 0.0;
	fs << "camera_p1" << 0.0;
	fs << "camera_p2" << 0.0;

	//fs << "\n";

	fs << "camera_width" << 1241;
	fs << "camera_height" << 376;
	
	//fs << "\n";

	fs << "camera_stereo_baseline" << 379.8145;

	Mat camera_matrix  = (Mat_<double>(3,3) << 707.0912, 0, 601.8873, 0, 707.0912, 183.1104, 0, 0, 1);

	Mat stereo_left_camera_matrix  = (Mat_<double>(3,4) << 707.0912, 0, 601.8873, 0, 0, 707.0912, 183.1104, 0, 0, 0, 1.0000, 0);

	Mat stereo_right_camera_matrix  = (Mat_<double>(3,4) << 707.0912, 0, 601.8873, -379.8145, 0, 707.0912, 183.1104, 0, 0, 0, 1.0000, 0);

	fs << "camera_matrix" << camera_matrix << "stereo_left_camera_matrix" <<stereo_left_camera_matrix << "stereo_right_camera_matrix" << stereo_right_camera_matrix;

	fs.release();	
	
	// read setting file
	FileStorage fs2("KITTI_04-12.yml", FileStorage::READ);
	
	if (!fs2.isOpened()){
		std::cerr << "failed to open setting file" <<std::endl;
		return 1;
	}

	double camera_fx = (double)fs2["camera_fx"];

	Mat camera_matrix_read, stereo_left_camera_matrix_read, stereo_right_camera_matrix_read;

	fs2["camera_matrix"] >> camera_matrix_read;
	fs2["stereo_left_camera_matrix"] >> stereo_left_camera_matrix_read;
	fs2["stereo_right_camera_matrix"] >> stereo_right_camera_matrix_read;
	
	std::cout << camera_matrix_read << std::endl;
	std::cout << stereo_left_camera_matrix_read << std::endl;
	std::cout << stereo_right_camera_matrix_read << std::endl;

	fs2.release();
	return 0;
}

/*
int main(int, char** argv)
{
	FileStorage fs("KITTI_00.yml", FileStorage::WRITE);

	fs << "camera_fx" << 718.856;
	fs << "camera_fy" << 718.856;
	fs << "camera_cx" << 607.1928;
	fs << "camera_cy" << 185.2157;

	//fs << "\n";

	fs << "camera_k1" << 0.0;
	fs << "camera_k2" << 0.0;
	fs << "camera_p1" << 0.0;
	fs << "camera_p2" << 0.0;

	//fs << "\n";

	fs << "camera_width" << 1241;
	fs << "camera_height" << 376;
	
	//fs << "\n";

	fs << "camera_stereo_baseline" << 386.1448;

	Mat camera_matrix  = (Mat_<double>(3,3) << 718.856, 0, 607.1928, 0, 718.8560, 185.2157, 0, 0, 1);

	Mat stereo_left_camera_matrix  = (Mat_<double>(3,4) << 718.8560, 0, 607.1928, 0, 0, 718.8560, 185.2157, 0, 0, 0, 1.0000, 0);

	Mat stereo_right_camera_matrix  = (Mat_<double>(3,4) << 718.8560, 0, 607.1928, -386.1448, 0, 718.8560, 185.2157, 0, 0, 0, 1.0000, 0);

	fs << "camera_matrix" << camera_matrix << "stereo_left_camera_matrix" <<stereo_left_camera_matrix << "stereo_right_camera_matrix" << stereo_right_camera_matrix;

	fs.release();	
	
	// read setting file
	FileStorage fs2("KITTI_00.yml", FileStorage::READ);

	double camera_fx = (double)fs2["camera_fx"];

	Mat camera_matrix_read, stereo_left_camera_matrix_read, stereo_right_camera_matrix_read;

	fs2["camera_matrix"] >> camera_matrix_read;
	fs2["stereo_left_camera_matrix"] >> stereo_left_camera_matrix_read;
	fs2["stereo_right_camera_matrix"] >> stereo_right_camera_matrix_read;
	
	std::cout << camera_matrix_read << std::endl;
	std::cout << stereo_left_camera_matrix_read << std::endl;
	std::cout << stereo_right_camera_matrix_read << std::endl;

	fs2.release();
	return 0;
}
*/
