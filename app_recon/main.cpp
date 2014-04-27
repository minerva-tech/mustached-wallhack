#include <stdio.h>
#include <string>
#include "display3d.h"
#ifdef _DEBUG
#pragma comment(lib, "opencv_core247d.lib")
#else
#pragma comment(lib, "opencv_core247.lib")
#endif

int main(int argc, char **argv)
{
	Display display;
	list<string> input_filenames;
	bool imaginary_chessboard = false;
	Chessboard chessboard;

	for(int i = 1; i < argc; ++i){
		if(string(argv[i]) == "-i")input_filenames.push_back(argv[++i]);
		else if(string(argv[i]) == "-this-is-a-chessboard-dude"){
			imaginary_chessboard = true;
			chessboard.cell_size = atof(argv[++i]);
			chessboard.width = atoi(argv[++i]);
			chessboard.height = atoi(argv[++i]);
		}
	}

	for(auto i = input_filenames.begin(); i != input_filenames.end(); ++i){
		try{
			FileStorage f = FileStorage(*i, FileStorage::READ);
		
			if(!f.isOpened())continue;

			FileNode node = f["calibration_time"];
			if(!node.empty()){
				Camera cam;
				Mat extrinsics;

				f["camera_matrix"] >> cam.intrin;
				f["distortion_coefficients"] >> cam.dist;
				f["extrinsic_parameters"] >> extrinsics;
		
				for(int i = 0; i < extrinsics.rows; ++i){
					Camera c = cam;
					Mat_<double> row = extrinsics.row(i);
					c.rvec.x = row(0, 0);
					c.rvec.y = row(0, 1);
					c.rvec.z = row(0, 2);
					c.tvec.x = row(0, 3);
					c.tvec.y = row(0, 4);
					c.tvec.z = row(0, 5);

					display.add(c);
				}
			}
		}catch(Exception ex){
			fputs(ex.what(), stderr);
		}
	}

	if(imaginary_chessboard)
		display.set(chessboard);

	display.run();
	return 0;
}
