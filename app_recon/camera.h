#pragma once
#include <opencv2/core/core.hpp>

struct Camera
{
	cv::Mat_<double> intrin;
	cv::Mat_<double> dist;
	cv::Point3d rvec;
	cv::Point3d tvec;
	cv::Mat image;
	int image_width;
	int image_height;
};
