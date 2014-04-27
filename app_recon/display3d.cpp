#define _USE_MATH_DEFINES
#include <math.h>
#include <map>
#include "gl/glut.h"
#include "display3d.h"

#define ZOOM_INCREMENT		0.1f
#define ROTATE_INCREMENT	0.5f

using namespace std;

static map<int, Display *> windows;

Display::Display(int width, int height)
	:min_pos(-1,-1,-1),
	max_pos(1,1,1),
	shift(0,0,4)
{
	int empty_argc = 0;
	char *empty_argv = 0;

	glutInitWindowSize(width, height);
	glutInitWindowPosition(100, 100);
	glutInit(&empty_argc, &empty_argv);
	glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH|GLUT_ALPHA);

	window_id = glutCreateWindow("display 3d");
	glutDisplayFunc(display_func);
	glutReshapeFunc(resize_func);
	glutKeyboardFunc(key_func);
	glutMouseFunc(mouse_click);
	glutMotionFunc(mouse_move);
	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable (GL_POLYGON_SMOOTH);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glBlendFunc (GL_SRC_ALPHA_SATURATE, GL_ONE);
	glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_DONT_CARE);

	glClearColor(0.90f, 0.90f, 0.90f, 0.0f);

	windows[window_id] = this;
}

Display::~Display()
{
	glutDestroyWindow(window_id);
	windows.erase(window_id);
}

#pragma region glut callbacks
void Display::display_func()
{
	windows[glutGetWindow()]->display();
}

void Display::resize_func(int width, int height)
{
	windows[glutGetWindow()]->resize(width, height);
}

void Display::key_func(unsigned char key, int x, int y)
{
	windows[glutGetWindow()]->key(key, Point2i(x,y));
}

void Display::mouse_click(int butt, int state, int x, int y)
{
	windows[glutGetWindow()]->mouse_click(butt, state, Point2i(x, y));
}

void Display::mouse_move(int x, int y)
{
	windows[glutGetWindow()]->mouse_move(Point2i(x, y));
}

#pragma endregion

void Display::add(const Camera &cam)
{
	cams.push_back(cam);
	update_range(cam.tvec);
}

void Display::add(const vector<Point3d> &points)
{
	objs.push_back(points);
}

void Display::set(const Chessboard &cb)
{
	chessboard = cb;
	update_range(Point3d(0,0,0));
	update_range(Point3d(chessboard.width*chessboard.cell_size, chessboard.height*chessboard.cell_size, 0));
}

void Display::update_range(const Point3d &pt)
{
	min_pos.x = min(min_pos.x, pt.x);
	min_pos.y = min(min_pos.y, pt.y);
	min_pos.z = min(min_pos.z, pt.z);

	max_pos.x = max(max_pos.x, pt.x);
	max_pos.y = max(max_pos.y, pt.y);
	max_pos.z = max(max_pos.z, pt.z);
}

// TODO: this function should be static
void Display::run()
{
	glutMainLoop();
}

void Display::display()
{
	int window_width = glutGet(GLUT_WINDOW_WIDTH);
	int window_height = glutGet(GLUT_WINDOW_HEIGHT);
	double width = max_pos.x - min_pos.x;//MAX(max_pos.x - min_pos.x, max_pos.y - min_pos.y);
	double height = max_pos.y - min_pos.y;//width;
	double centerx = (min_pos.x + max_pos.x)/2;// + shift_x;
	double centery = (min_pos.y + max_pos.y)/2;// + shift_y;
	double centerz = (min_pos.z + max_pos.z)/2;
	double window_ar = double(window_width)/double(window_height);
	double world_ar = width/height;

	if(window_ar < world_ar){
		height = width / window_ar;
	}else{
		width = window_ar * height;
	}

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90, window_ar, 0.001, 100);
	gluLookAt(centerx, centery, max_pos.z + shift.z, centerx, centery, centerz, 0, 1, 0);
	glViewport(0,0,window_width,window_height);
	glMatrixMode(GL_MODELVIEW);

	Point3f dist = min_pos + ((max_pos - min_pos)*0.5f);

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glTranslated(dist.x, dist.y, dist.z);
	glRotated(rot.x, 1, 0, 0);
	glRotated(rot.y, 0, 1, 0);
	glRotated(rot.z, 0, 0, 1);
	glTranslated(-dist.x, -dist.y, -dist.z);

	draw_axis();

	for(auto c = cams.begin(); c != cams.end(); ++c)draw_camera(*c);

	for(auto o = objs.begin(); o != objs.end(); ++o)draw_object(*o);

	if(chessboard.cell_size > 0)
		draw_chessboard(chessboard);

	glFlush();
	glutSwapBuffers();
}

void Display::draw_axis()
{
	float prev_width;
	glGetFloatv(GL_LINE_WIDTH, &prev_width);
	glLineWidth(3);

	glColor3f(0.90f, 0.1f, 0.1f);
	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	glVertex3f(1,0,0);
	glEnd();

	glColor3f(0.1f, 0.90f, 0.1f);
	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	glVertex3f(0,1,0);
	glEnd();

	glColor3f(0.1f, 0.1f, 0.90f);
	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	glVertex3f(0,0,1);
	glEnd();

	glLineWidth(prev_width);
}

void Display::draw_camera(const Camera &cam)
{
	glPushMatrix();
	double a = norm(cam.rvec);
	glRotated(a/M_PI*180.0, cam.rvec.x, cam.rvec.y, cam.rvec.z);
	glTranslated(cam.tvec.x, cam.tvec.y, cam.tvec.z);

	glColor3f(0,0,0);
	glBegin(GL_LINE_STRIP);
	glVertex3d(-1,-1,0);
	glVertex3f(-1, 1,0);
	glVertex3f(1,1,0);
	glVertex3f(1,-1,0);
	glVertex3d(-1,-1,0);
	glEnd();

	glPopMatrix();
}

void Display::draw_object(const vector<Point3d> &obj)
{
}

void Display::draw_chessboard(const Chessboard &cb)
{
	glBegin(GL_QUADS);

	for(int x = 0; x < cb.width + 1; ++x)for(int y = 0; y < cb.height + 1; ++y){
		if((x + y) & 1)glColor3f(1,1,1);
		else glColor3f(0,0,0);
		glVertex3d(x*cb.cell_size, y*cb.cell_size, 0);
		glVertex3d((x+1)*cb.cell_size, y*cb.cell_size, 0);
		glVertex3d((x+1)*cb.cell_size, (y+1)*cb.cell_size, 0);
		glVertex3d(x*cb.cell_size, (y+1)*cb.cell_size, 0);
	}

	glEnd();
}

void Display::resize(int width, int height)
{
}

void Display::key(unsigned char key, const Point2i &pos)
{
	switch(key){
	case 27:
	case 'q':
		// TODO: this is a wrong way to exit
		glutDestroyWindow(window_id);
		break;
	}
}

void Display::mouse_click(int butt, int state, const Point2i &pos)
{
	switch(butt & 0x0f){
	case GLUT_LEFT_BUTTON:
		if(state == GLUT_DOWN){
			is_dragging = true;
			drag_start = pos;
			rot_start = rot;
		}
		else if(state == GLUT_UP){
			is_dragging = false;
		}

		break;
	case 3: // mouse wheel up
		shift.z -= ZOOM_INCREMENT;
		break;
	case 4: // mouse wheel down
		shift.z += ZOOM_INCREMENT;
		break;
	default:
		return;
	}
	glutPostRedisplay();
}

void Display::mouse_move(const Point2i &pos)
{
	if(is_dragging){
		Point2i delta = pos - drag_start;
		rot = rot_start + Point3d(delta.y*ROTATE_INCREMENT, 0, delta.x*ROTATE_INCREMENT);
		glutPostRedisplay();
	}
}
