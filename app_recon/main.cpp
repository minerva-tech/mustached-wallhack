#include <stdio.h>
#include <string>
#include <stdarg.h>
#include "display3d.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

#ifdef _MSC_VER
#ifdef _DEBUG
#pragma comment(lib, "opencv_core247d.lib")
#pragma comment(lib, "opencv_highgui247d.lib")
#else
#pragma comment(lib, "opencv_core247.lib")
#pragma comment(lib, "opencv_highgui247.lib")
#endif
#endif

int draw_cam_calibration(const FileStorage &f, VideoCapture &capture, const Camera &cam)
{
    Display display;
    Mat extrinsics;

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
            capture.set(CV_CAP_PROP_POS_FRAMES, vid_frames(i));
            capture.read(c.image);
        }

        display.add(c);
    }
    printf("\nsetup %d cameras\n", i);

    display.run();

    return 0;
}

string sprintf(const char *format, ...)
{
    va_list list;
    va_start(list, format);
    char buf[1024];
    vsnprintf(buf, sizeof(buf), format, list);
    va_end(list);
    return string(buf);
}

int recon_stereo(const Mat *images, const Camera &cam)
{
    BRISK brisk;
    vector<KeyPoint> keypoints[2];
    Mat desc[2];

    for(int i = 0; i < 2; ++i){
        brisk(images[i], Mat(), keypoints[i], desc[i]);
        Mat kp_image;
        drawKeypoints(images[i], keypoints[i], kp_image, DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        //imshow(sprintf("keypoints %d", i), kp_image);
        imwrite(sprintf("/tmp/keypoints%d.jpg", i), kp_image);
    }

    return 0;
}

int try_recon(VideoCapture &capture, const Camera &cam)
{
    Mat images[2];

    if(!capture.read(images[0])){
        printf("empty video\n");
        return -1;
    }

    if(capture.read(images[1])){
        printf("reconstruct pair...\n");
        recon_stereo(images, cam);
        images[0] = images[1];
    }

    return 0;
}

int main(int argc, char **argv)
{
	string camera_filename;
	string video_filename;
    bool is_draw_cam_calibration = false;
	VideoCapture capture;

    setbuf(stdout, 0);

	for(int i = 1; i < argc; ++i){
        if(false);
        else if(string(argv[i]) == "-i")camera_filename = argv[++i];
		else if(string(argv[i]) == "-V")video_filename = argv[++i];
        else if(string(argv[i]) == "-draw-cam-calibration")is_draw_cam_calibration = true;
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

			f["image_width"] >> cam.image_width;
			f["image_height"] >> cam.image_height;
			f["camera_matrix"] >> cam.intrin;
			f["distortion_coefficients"] >> cam.dist;

            if(is_draw_cam_calibration){
                draw_cam_calibration(f, capture, cam);
            }
            else if(capture.isOpened() && !cam.intrin.empty()){
                try_recon(capture, cam);
            }
            else{
                printf("no video or camera intrinsics unknown. skip reconstruction\n");
                return -2;
            }
		}
	}catch(Exception ex){
		fputs(ex.what(), stderr);
	}

    return 0;
}
