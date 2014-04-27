#include <stdio.h>
#include <string>
#include "display3d.h"
#include "opencv2/highgui/highgui.hpp"

#ifdef _DEBUG
#pragma comment(lib, "opencv_core247d.lib")
#pragma comment(lib, "opencv_highgui247d.lib")
#else
#pragma comment(lib, "opencv_core247.lib")
#pragma comment(lib, "opencv_highgui247.lib")
#endif

int main(int argc, char **argv)
{
	Display display;
	list<string> input_filenames;
	string video_filename;
	bool imaginary_chessboard = false;
	Chessboard chessboard;
	VideoCapture capture;

	for(int i = 1; i < argc; ++i){
		if(string(argv[i]) == "-i")input_filenames.push_back(argv[++i]);
		else if(string(argv[i]) == "-this-is-a-chessboard-dude"){
			imaginary_chessboard = true;
			chessboard.cell_size = atof(argv[++i]);
			chessboard.width = atoi(argv[++i]);
			chessboard.height = atoi(argv[++i]);
		}
		else if(string(argv[i]) == "-V")video_filename = argv[++i];
	}

	if(video_filename.length() > 0)
		capture.open(video_filename);

	for(auto i = input_filenames.begin(); i != input_filenames.end(); ++i){
		try{
			FileStorage f = FileStorage(*i, FileStorage::READ);
		
			if(!f.isOpened())continue;

			FileNode node = f["calibration_time"];
			if(!node.empty()){
				printf("camera calibration file!\n");
				Camera cam;
				Mat extrinsics;

				f["image_width"] >> cam.image_width;
				f["image_height"] >> cam.image_height;
				f["camera_matrix"] >> cam.intrin;
				f["distortion_coefficients"] >> cam.dist;
				f["extrinsic_parameters"] >> extrinsics;

				Mat_<int> vid_frames;
				if(!f["video_frame_numbers"].empty())f["video_frame_numbers"] >> vid_frames;				
		
				int i;
				bool have_pictures = (!vid_frames.empty() && capture.isOpened());
				for(i = 0; i < extrinsics.rows; ++i){
					Camera c = cam;
					Mat_<double> row = extrinsics.row(i);
					c.rvec.x = row(0, 0);
					c.rvec.y = row(0, 1);
					c.rvec.z = row(0, 2);
					c.tvec.x = row(0, 3);
					c.tvec.y = row(0, 4);
					c.tvec.z = row(0, 5);

					if(have_pictures){
						printf("\rloading picture %d/%d", i, extrinsics.rows);
						Mat image;
						capture.set(CV_CAP_PROP_POS_FRAMES, vid_frames(i));
						capture.read(image);
						//cvtColor(image, c.image, 
						c.image = image;
					}

					display.add(c);
				}
				printf("\nsetup %d cameras\n", i);
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
