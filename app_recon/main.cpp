#include <stdio.h>
#include "display3d.h"
#ifdef _DEBUG
#pragma comment(lib, "opencv_core247d.lib")
#endif

int main(int argc, char **argv)
{
	Display display;

	display.run();
	return 0;
}
