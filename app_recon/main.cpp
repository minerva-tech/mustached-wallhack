#include <stdio.h>
#include <string>
#include "display3d.h"
#include "opencv2/highgui/highgui.hpp"

#ifdef _DEBUG
//#pragma comment(lib, "opencv_core247d.lib")
//#pragma comment(lib, "opencv_highgui247d.lib")
#pragma comment(lib, "opencv_world310d.lib")
#else
#pragma comment(lib, "opencv_core247.lib")
#pragma comment(lib, "opencv_highgui247.lib")
#endif

int main(int argc, char **argv)
{
	Display display;
	string camera_filename;
	string video_filename;
	bool imaginary_chessboard = false;
	VideoCapture capture;
	Camera camera;

	for(int i = 1; i < argc; ++i){
		if(string(argv[i]) == "-i")camera_filename = argv[++i];
		else if(string(argv[i]) == "-V")video_filename = argv[++i];
	}

	if(video_filename.length() > 0)
		capture.open(video_filename);

	try{
		FileStorage f = FileStorage(camera_filename, FileStorage::READ);
		
		if(!f.isOpened()){
			fprintf(stderr, "fail to open file %s\n", camera_filename.c_str());
			return -1;
		}

		FileNode node = f["calibration_time"];
		if(!node.empty()){
			printf("reading camera calibration file\n");
			Camera cam;
			Mat extrinsics;

			f["image_width"] >> cam.image_width;
			f["image_height"] >> cam.image_height;
			f["camera_matrix"] >> cam.intrin;
			f["distortion_coefficients"] >> cam.dist;

			camera = cam;

			if(!f["board_width"].empty()){
				Chessboard board;
				f["board_width"] >> board.width;
				f["board_height"] >> board.height;
				f["square_size"] >> board.cell_size;
				display.set(board);
				printf("\tfound board %dx%d cell %f\n", board.width, board.height, board.cell_size);
			}

			f["extrinsic_parameters"] >> extrinsics;

			Mat_<int> vid_frames;
			if(!f["video_frame_numbers"].empty()){
				f["video_frame_numbers"] >> vid_frames;
				printf("\tvideo frame numbers found (%d entries)\n", vid_frames.rows);
			}
		
			int i;
			bool have_pictures = (!vid_frames.empty() && capture.isOpened());
			
			if(have_pictures){
				printf("\talso video exist, loading pictures\n");
			}

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
					printf("\r\tloading picture %d/%d", i, extrinsics.rows);
					Mat image;
					capture.set(CV_CAP_PROP_POS_FRAMES, vid_frames(i));
					capture.read(c.image);
				}

				display.add(c);
			}
			printf("\nsetup %d cameras\n", i);
		}
	}catch(Exception ex){
		fputs(ex.what(), stderr);
	}

	if(capture.isOpened() && !camera.intrin.empty()){

	}
	else{
		printf("no video or camera intrinsics unknown. skip reconstruction\n");
		return -2;
	}

	display.run();
	return 0;
}
