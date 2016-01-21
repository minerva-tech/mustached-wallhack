#include <stdio.h>
#include <string>
#include <stdarg.h>
#include <iostream>
#include "display3d.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#ifdef _MSC_VER
#ifdef _DEBUG
#pragma comment(lib, "opencv_world310d.lib")
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

vector<KeyPoint> reorder(const vector<KeyPoint> &keypoints, const vector<DMatch> &matches)
{
    vector<KeyPoint> filtered(matches.size());

    for(size_t i = 0; i < matches.size(); ++i){
        filtered[i] = keypoints[matches[i].trainIdx];
    }
    return filtered;
}

template <class T>
vector<T> filter(const vector<T> &keypoints, const Mat &mask)
{
    vector<T> filtered;
    for(size_t i = 0; i < keypoints.size(); ++i){
        if(mask.at<bool>(i))filtered.push_back(keypoints[i]);
    }
    return filtered;
}

Mat find_homography(const vector<KeyPoint> *keypoints, const vector<DMatch> &matches, Mat &mask)
{
    int n = keypoints[0].size();

    Mat points[2] = {Mat(n, 1, CV_32FC2), Mat(n, 1, CV_32FC2)};
    for(int j = 0; j < n; ++j){
        points[0].at<Point2f>(j, 0) = keypoints[0][j].pt;
        points[1].at<Point2f>(j, 0) = keypoints[1][matches[j].trainIdx].pt;
    }

    Mat homo = findHomography(points[0], points[1], mask, CV_RANSAC);
    cout << "homography" << endl << homo << endl;

    return homo;
}

void keypoints_optical_flow(const Mat *images, vector<KeyPoint> *keypoints, vector<DMatch> &matches)
{
    Mat err;
    SurfFeatureDetector detector;
    Mat desc[2];

    detector(images[0], Mat(), keypoints[0], desc[0]);

    keypoints[1].resize(keypoints[0].size());
    matches.resize(keypoints[0].size());

    vector<Point2f> points[2];
    points[0].resize(keypoints[0].size());
    for(size_t i = 0; i < keypoints[0].size(); ++i){
        points[0][i] = keypoints[0][i].pt;
    }

    vector<Mat> pyramids[2];
    for(int i = 0; i < 2; ++i)
        buildOpticalFlowPyramid(images[i], pyramids[i], Size(21, 21), 4);

    Mat mask;
    calcOpticalFlowPyrLK(pyramids[0], pyramids[1], points[0], points[1], mask, err);

    for(size_t i = 0; i < keypoints[0].size(); ++i){
        keypoints[1][i].pt = points[1][i];
        matches[i] = DMatch(i, i, 0);
    }

    for(int i = 0; i < 2; ++i){
        Mat im_keypoints;
        drawKeypoints(images[i], keypoints[i], im_keypoints, DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        imshow(sprintf("keypoints %d", i), im_keypoints);
        imwrite(sprintf("/tmp/keypoints%d.jpg", i), im_keypoints);
    }
}

void keypoints_bf(const Mat *images, vector<KeyPoint> *keypoints, vector<DMatch> &matches)
{
    Mat desc[2];
    SurfFeatureDetector detector(400);

    for(int i = 0; i < 2; ++i){
        detector(images[i], Mat(), keypoints[i], desc[i]);
        printf("find %d keypoints\n", (int)keypoints[i].size());

        Mat im_keypoints;
        drawKeypoints(images[i], keypoints[i], im_keypoints, DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        /*Mat canny;
        vector<Vec4i> lines;
        Canny(images[i], canny, 10, 100);
        //cvtColor(canny, grayscale, CV_GRAY2BGR );
        HoughLinesP(canny, lines, 1, CV_PI/180, 100, 30, 10);
        for(size_t i = 0; i < lines.size(); ++i)
            line(im_keypoints, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3);
        */

        imshow(sprintf("keypoints %d", i), im_keypoints);
        imwrite(sprintf("/tmp/keypoints%d.jpg", i), im_keypoints);
    }

    //FlannBasedMatcher matcher;
    BFMatcher matcher;
    matcher.match(desc[0], desc[1], matches);
}

int recon_stereo(const Mat *images, const Camera &cam)
{
    vector<KeyPoint> keypoints[2];
    vector<DMatch> matches;

    keypoints_bf(images, keypoints, matches);
    //keypoints_optical_flow(images, keypoints, matches);

    Mat mask;
    Mat homo = find_homography(keypoints, matches, mask);

    Mat im_matches;
    drawMatches(images[0], keypoints[0], images[1], keypoints[1], matches, im_matches, Scalar::all(-1), Scalar::all(-1), mask);
    imshow("matches", im_matches);
    imwrite("/tmp/matches.jpg", im_matches);

    keypoints[1] = reorder(keypoints[1], matches);
    keypoints[0] = filter(keypoints[0], mask);
    keypoints[1] = filter(keypoints[1], mask);

    {
        int n = keypoints[0].size();

        Mat points[2] = {Mat(n, 2, CV_32FC1), Mat(n, 2, CV_32FC1)};

        for(int i = 0; i < 2; ++i)for(int j = 0; j < n; ++j){
            points[i].at<float>(j, 0) = keypoints[i][j].pt.x;
            points[i].at<float>(j, 1) = keypoints[i][j].pt.y;
        }

        Mat fund = findFundamentalMat(points[0], points[1]);
        cout << "fundamental matrix" << endl << fund << endl;

        Mat ess = cam.intrin.t() * fund * cam.intrin;
        cout << "essensial matrix" << endl << ess << endl;
    }

    waitKey();

    return 0;
}

bool read(VideoCapture &capture, const Camera &cam, Mat &dst)
{
    Mat raw;
    Mat image;
    if(capture.read(raw)){
        undistort(raw, image, cam.intrin, cam.dist);
        bilateralFilter(image, dst, -1, 10, 10);
        return true;
    }
    return false;
}

int try_recon(VideoCapture &capture, const Camera &cam)
{
    Mat images[2];

    if(!read(capture, cam, images[0])){
        printf("empty video\n");
        return -1;
    }

    if(read(capture, cam, images[1])){
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
