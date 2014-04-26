#pragma once
#include <opencv2/core/core.hpp>
#include <list>

using namespace cv;
using namespace std;

class Camera
{
public:
	Camera(const Mat &intrinsics, const Mat &distorsion, const Mat &rvec, const Mat &tvec, const Mat &image);
private:
	Mat intrin;
	Mat dist;
	Mat rvec;
	Mat tvec;
	Mat image;
};

class Display
{
public:
	Display(int width = 720, int height = 576);
	~Display();

	void add(const Camera &cam);
	void add(const vector<Point3f> &points);

	void run();
private:
	static void display_func();
	static void resize_func(int width, int height);
	static void key_func(unsigned char key, int x, int y);
	static void mouse_move();
	static void mouse_click();

	void display();
	void resize(int width, int height);
	void key(unsigned char key, int x, int y);

	int window_id;

	list<Camera> cams;
	list<vector<Point3f>> objs;
};
