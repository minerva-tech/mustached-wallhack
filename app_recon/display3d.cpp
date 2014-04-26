#include <map>
#include "gl/glut.h"
#include "display3d.h"

using namespace std;

static map<int, Display *> windows;

Display::Display(int width, int height)
{
	int empty_argc = 0;
	char *empty_argv = 0;

	glutInitWindowSize(width, height);
	glutInitWindowPosition(100, 100);
	glutInit(&empty_argc, &empty_argv);
	glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);

	window_id = glutCreateWindow("display 3d");
	glutDisplayFunc(display_func);
	glutReshapeFunc(resize_func);
	glutKeyboardFunc(key_func);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_POINT_SMOOTH);

	windows[window_id] = this;
}

Display::~Display()
{
	glutDestroyWindow(window_id);
	windows.erase(window_id);
}

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
	windows[glutGetWindow()]->key(key, x, y);
}

// TODO: this function should be static
void Display::run()
{
	glutMainLoop();
}

void Display::display()
{
	glutSwapBuffers();
}

void Display::resize(int width, int height)
{
}

void Display::key(unsigned char key, int x, int y)
{
	switch(key){
	case 27:
	case 'q':
		// TODO: this is a wrong way to exit
		glutDestroyWindow(window_id);
		break;
	}
}
