#pragma once
#include <GL/glut.h>
#include <opencv2/core/core.hpp>
#include <list>
#include "../librecon/camera.h"

using namespace cv;
using namespace std;

class DisplayedCamera: public Camera
{
public:
	DisplayedCamera();
	DisplayedCamera(const Camera &cam);

	void draw(bool is_simple) const;
private:
	GLuint texid;
};

struct Chessboard
{
	Chessboard():cell_size(0), width(0), height(0){}

	bool valid(){ return cell_size > 0; }

	double cell_size;
	int width;
	int height;
};

class Display
{
public:
	Display(int width = 720, int height = 576);
	~Display();

	void add(const Camera &cam);
	void add(const vector<Point3d> &points);
	void set(const Chessboard &cb);

	void run();
private:
	static void display_func();
	static void resize_func(int width, int height);
	static void key_func(unsigned char key, int x, int y);
	static void mouse_click(int butt, int state, int x, int y);
	static void mouse_move(int x, int y);

	static void draw_axis();
	static void draw_object(const vector<Point3d> &obj);
	//void draw_camera(const DisplayedCamera &cam);
	static void draw_chessboard(const Chessboard &cb);

	void display();
	void resize(int width, int height);
	void key(unsigned char key, const Point2i &pos);
	void mouse_click(int butt, int state, const Point2i &pos);
	void mouse_move(const Point2i &pos);

	void update_range(const Point3d &pt);

	int window_id;

	bool is_dragging;
	Point2i drag_start;
	Point3d rot_start;

	bool is_simple_mode;

	list<DisplayedCamera> cams;
    list<vector<Point3d>> objs;
	Chessboard chessboard;
	
	Point3d min_pos;
	Point3d max_pos;

	Point3d shift;
	Point3d rot;
};
