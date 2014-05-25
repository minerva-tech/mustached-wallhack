#include "pchbarrier.h"
#include "settings.h"
#include "defs.h"
#include "iscanner.h"
#include "scanner.h"
#include "exception.h"

#ifdef _DEBUG
#pragma comment(lib, "opencv_core249d.lib")
#pragma comment(lib, "opencv_highgui249d.lib")
#pragma comment(lib, "opencv_flann249d.lib")
#pragma comment(lib, "opencv_imgproc249d.lib")
#pragma comment(lib, "opencv_features2d249d.lib")
#pragma comment(lib, "opencv_calib3d249d.lib")
#pragma comment(lib, "opencv_video249d.lib")
#pragma comment(lib, "zlibd.lib")
#pragma comment(lib, "libtiffd.lib")
#pragma comment(lib, "libjpegd.lib")
#pragma comment(lib, "libpngd.lib")
#pragma comment(lib, "libjasperd.lib")
#pragma comment(lib, "IlmImfd.lib")
#else
#pragma comment(lib, "opencv_core249.lib")
#pragma comment(lib, "opencv_highgui249.lib")
#pragma comment(lib, "opencv_calib3d249.lib")
#pragma comment(lib, "opencv_flann249.lib")
#pragma comment(lib, "opencv_imgproc249.lib")
#pragma comment(lib, "opencv_features2d249.lib")
#pragma comment(lib, "opencv_video249.lib")
#pragma comment(lib, "zlib.lib")
#pragma comment(lib, "libtiff.lib")
#pragma comment(lib, "libjpeg.lib")
#pragma comment(lib, "libpng.lib")
#pragma comment(lib, "libjasper.lib")
#pragma comment(lib, "IlmImf.lib")
#endif

#pragma comment(lib, "vfw32.lib")

static std::vector<std::vector<cv::Point3f> > fill_obj_points(int w, int h)
{
	std::vector<std::vector<cv::Point3f> > ms;

	ms.push_back(std::vector<cv::Point3f>());

	std::vector<cv::Point3f>& m = ms.back();

	m.resize((w-1)*(h-1));

	const double x_step = 1./(double)w;
	const double y_step = 1./(double)h;

	const double x_off = 0;
	const double y_off = 0;

	for(uint32_t i=0; i<m.size(); i++) {
		const double x = i % (w-1);
		const double y = i / (h-1);

		m[i].x = (float)(x * x_step) + x_off;
		m[i].y = (float)(y * y_step) + y_off;
		m[i].z = 0.;
	}

#if !(STEREO_CALIBRATE)
	ms.push_back(ms.back());
#endif

	return ms;
}

static void write_scene(std::ofstream& fstr, const cv::Mat& pts, const std::vector<cv::Vec3b> colors) // todo: the same as draw_cube
{
	const float d = 0.005;

	for (int i = 0; i < pts.cols; i++) {
		const float x = pts.at<float>(X, i);// / pts.at<float>(3, i);
		const float y = pts.at<float>(Y, i);// / pts.at<float>(3, i);
		const float z = pts.at<float>(Z, i) / 100.;// / pts.at<float>(3, i);

		const int color[3] = { colors[i][0], colors[i][1], colors[i][2] };

		fstr << "v " << x - d << " " << y - d << " " << z - d << " " << color[0] << " " << color[1] << " " << color[2] << std::endl; // -8
		fstr << "v " << x + d << " " << y - d << " " << z - d << " " << color[0] << " " << color[1] << " " << color[2] << std::endl; // -7
		fstr << "v " << x + d << " " << y + d << " " << z - d << " " << color[0] << " " << color[1] << " " << color[2] << std::endl; // -6
		fstr << "v " << x - d << " " << y + d << " " << z - d << " " << color[0] << " " << color[1] << " " << color[2] << std::endl; // -5

		fstr << "v " << x - d << " " << y - d << " " << z + d << " " << color[0] << " " << color[1] << " " << color[2] << std::endl; // -4
		fstr << "v " << x + d << " " << y - d << " " << z + d << " " << color[0] << " " << color[1] << " " << color[2] << std::endl; // -3
		fstr << "v " << x + d << " " << y + d << " " << z + d << " " << color[0] << " " << color[1] << " " << color[2] << std::endl; // -2
		fstr << "v " << x - d << " " << y + d << " " << z + d << " " << color[0] << " " << color[1] << " " << color[2] << std::endl; // -1

		fstr << "f -8 -5 -6 -7" << std::endl;
		fstr << "f -4 -3 -2 -1" << std::endl;

		fstr << "f -8 -7 -3 -4" << std::endl;
		fstr << "f -6 -5 -1 -2" << std::endl;

		fstr << "f -5 -8 -4 -1" << std::endl;
		fstr << "f -7 -6 -2 -3" << std::endl;
	}
}

static uint32_t bit_n(uint32_t a)
{
	a = a - ((a >> 1) & 0x55555555);
	a = (a & 0x33333333) + ((a >> 2) & 0x33333333);
	return (((a + (a >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

static int hamming_dist(const cv::Mat& a, const cv::Mat& b)
{
	int dist = 0;

	for (int i = 0; i < a.cols; i++) {
		dist += bit_n(a.at<uint8_t>(i) ^ b.at<uint8_t>(i));
	}

	return dist;
}

cv::Mat g_projPt_base(2, 9*6, CV_32F);
cv::Mat g_projPt_cur(2, 9*6, CV_32F);

static void recon_scene(std::ofstream& fstr, const cv::Mat& trans, const cv::Mat& rot, const cv::Mat& base_fr, const cv::Mat& cur_fr,
	const cv::Mat& P0, const cv::Mat& P1)
{
	const int DIST_THRES = 90;

	// We don't need K matrix as internal parameters are the same for all frames, right?

	cv::BRISK brisk(10);

	std::vector<cv::KeyPoint> keypoints_base;
	std::vector<cv::KeyPoint> keypoints_cur;
	std::vector<cv::KeyPoint> keypoints_base_;
	std::vector<cv::KeyPoint> keypoints_cur_;
	cv::Mat desc_base;
	cv::Mat desc_cur;

	brisk(base_fr, cv::Mat(), keypoints_base, desc_base, false);
	brisk(cur_fr, cv::Mat(), keypoints_cur, desc_cur, false);
	/*
	cv::Mat projPt_base(2, keypoints_base.size(), CV_32F);
//	cv::Mat projPt_cur(2, keypoints_cur.size(), CV_32F);

	//	std::cout << "keypoints: " << keypoints_base.size() << " " << keypoints_cur.size() << std::endl;

	cv::Mat keypoints_moved;
	cv::Mat status;
	cv::Mat err;

	std::vector<cv::Point2f> keypoints_base_2d;
	keypoints_base_2d.reserve(keypoints_base.size());

	for (int i = 0; i < keypoints_base.size(); i++)
		keypoints_base_2d.push_back(keypoints_base[i].pt);

	cv::calcOpticalFlowPyrLK(base_fr, cur_fr, keypoints_base_2d, keypoints_moved, status, err);

	int matches = 0;
	*/
	/*
	for (size_t i = 0; i < keypoints_base.size(); i++) {
		for (size_t j = 0; j < keypoints_cur.size(); j++) {
			const bool match = hamming_dist(desc_base.row(i), desc_cur.row(j)) < DIST_THRES;
			if (match) {
				projPt_base.at<float>(X, matches) = keypoints_base[i].pt.x;
				projPt_base.at<float>(Y, matches) = keypoints_base[i].pt.y;
				keypoints_base_.push_back(keypoints_base[i]);

				projPt_cur.at<float>(X, matches) = keypoints_cur[j].pt.x;
				projPt_cur.at<float>(Y, matches) = keypoints_cur[j].pt.y;
				keypoints_cur_.push_back(keypoints_cur[j]);

				matches++;

				break;
			}
		}
	}

	matches = keypoints_moved.cols;

	*/

	cv::Mat projPt_base = g_projPt_base;
	cv::Mat projPt_cur = g_projPt_cur;
	
	std::vector<cv::Vec3b> colors;

	std::fill_n(std::inserter(colors, colors.begin()), projPt_base.cols, cv::Vec3b(255, 255, 255));

	/*
	cv::Mat projPt_cur(2, matches, CV_32F);

	std::cout << "matches: " << matches << std::endl;

	std::vector<cv::Vec3b> colors;

	for (int i = 0; i < keypoints_moved.cols; i++) {
		const int x = keypoints_base[i].pt.x;
		const int y = keypoints_base[i].pt.y;

		colors.push_back(base_fr.at<cv::Vec3b>(y, x));

		projPt_base.at<float>(X, i) = x;
		projPt_base.at<float>(Y, i) = y;

		projPt_cur.at<float>(X, i) = keypoints_moved.at<cv::Point2f>(0, i).x;
		projPt_cur.at<float>(Y, i) = keypoints_moved.at<cv::Point2f>(0, i).y;
	}
	
	projPt_base = projPt_base.colRange(0, matches);
	projPt_cur = projPt_cur.colRange(0, matches);
	*/

	/*
////
	cv::Mat img;
	static int fr_cnt = 0;
	cv::drawKeypoints(base_fr, keypoints_base_, img);
	cv::imwrite(std::string("points_base") + boost::lexical_cast<std::string>(fr_cnt)+".jpg", img);
	img;
	cv::drawKeypoints(cur_fr, keypoints_cur_, img);
	cv::imwrite(std::string("points_cur") + boost::lexical_cast<std::string>(fr_cnt)+".jpg", img);
	fr_cnt++;
////
	*/


/*
	cv::Mat projMat_base(cv::Mat::zeros(3, 4, CV_32F));
	cv::Mat projMat_cur(cv::Mat::zeros(3, 4, CV_32F));

	projMat_base.at<float>(0, 0) = projMat_base.at<float>(1, 1) = projMat_base.at<float>(2, 2) = 1.f; // .diag=ones() bla-bla-bla

	std::cout << rot << std::endl;
	std::cout << trans << std::endl;

	projMat_base.copyTo(projMat_cur);

	rot.copyTo(projMat_cur.colRange(0, 3));
	trans.copyTo(projMat_cur.colRange(3, 4));
*/
	cv::Mat out;// (4, matches, CV_32F);
//	std::cout << out << std::endl;

/*	std::cout << projMat_base << std::endl;
	std::cout << projMat_cur << std::endl;
	
	std::cout << projPt_base << std::endl;
	std::cout << projPt_cur << std::endl;
	*/
//	cv::triangulatePoints(projMat_base, projMat_cur, projPt_base, projPt_cur, out);
	cv::triangulatePoints(P0, P1, projPt_base, projPt_cur, out);

//	std::cout << out << std::endl;

	write_scene(fstr, out, colors);
}

IScanner::Impl::Impl(const Settings& settings) :
	g_obj_points(fill_obj_points(settings.n_chessboard_cols, settings.n_chessboard_rows)),
	m_settings(settings),
	m_sum_rot(cv::Mat::eye(3, 3, CV_64F))
{

}

void IScanner::Impl::process()
{
	open();

	m_settings.n_end_frame = std::min(m_settings.n_end_frame, m_frm_cnt);

	cv::Mat pos = cv::Mat::zeros(3, 1, CV_64F);

    std::vector <cv::Point2f> corners;

    corners.resize((m_settings.n_chessboard_cols-1) * (m_settings.n_chessboard_rows-1));

    const CvSize pat_size = cvSize(m_settings.n_chessboard_cols-1, m_settings.n_chessboard_rows-1);

	int found_cnt = 0;

	for (int fr_cnt = 0; fr_cnt < m_settings.n_end_frame; fr_cnt++) {
	    const bool found_fast = findChessboardCorners(m_images[fr_cnt], pat_size, corners, cv::CALIB_CB_FAST_CHECK);

        if (found_fast) {
            const bool found = findChessboardCorners(m_images[fr_cnt], pat_size, corners);

            if (found) {
                drawChessboardCorners(m_images[fr_cnt], pat_size, corners, found);

				cv::imwrite(std::string("board") + boost::lexical_cast<std::string>(fr_cnt)+".jpg", m_images[fr_cnt]);

                m_frames.push_back(FrameData());

                m_frames.back().corners.push_back(corners);
                m_frames.back().idx = found_cnt;
				m_frames.back().img = m_images[fr_cnt];

				found_cnt++;

                std::cout << "frame #" << fr_cnt << " chessboard was found." << std::endl;
            } else {
				std::cout << "frame #" << fr_cnt << " chessboard WASN'T found (slow)." << std::endl;
            }
        } else {
            std::cout << "frame #" << fr_cnt << " chessboard WASN'T found (fast)." << std::endl;
        }
    }

	m_images.clear();

	if (m_frames.empty()) {
		close();
		return;
	}

    cv::Mat camera[2];
    std::vector<float> dist[2];

    dist[0].resize(9);
    dist[1].resize(9);

   	cv::Mat rot;
	cv::Mat trans;
	cv::Mat E;
	cv::Mat F;

	std::vector<cv::Mat> rots;
	std::vector<cv::Mat> transes;

    const int len = (m_settings.n_chessboard_cols-1) * (m_settings.n_chessboard_rows-1);

    const double ANGLE_THRES = CV_PI / 4.; // if angle between two camera positions is bigger than this, chessboard should be rotated. I suppose that there is no large jumps in video sequence. All neighbor frames are close each other.

    for(auto base_fr = m_frames.begin(); base_fr+1 != m_frames.end(); ++base_fr) {
        for(auto cur_fr = base_fr+1; cur_fr != m_frames.end(); ++cur_fr) {
            int tries = 0;

//			cv::Mat initial_cam = cv::initCameraMatrix2D(g_obj_points, base_fr->corners, cv::Size(base_fr->img.cols, base_fr->img.rows), 0);
//			std::cout << initial_cam << std::endl;
//			initial_cam.copyTo(camera[0]);

            while (tries++ < 4) {
#if STEREO_CALIBRATE
//                double err = cv::stereoCalibrate(g_obj_points, base_fr->corners, cur_fr->corners, camera[0], dist[0], camera[1], dist[1], cvSize(1, len), rot, trans, E, F, 
//    		        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 1e-6), CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);
#else
/*				std::vector<cv::Mat> corners;

				corners.push_back(cv::Mat(2, base_fr->corners.size(), CV_32F));
				for (int i = 0; i < base_fr->corners[0].size(); i++) {
					corners.back().at<float>(X, i) = base_fr->corners[0][i].x;
					corners.back().at<float>(Y, i) = base_fr->corners[0][i].y;
				}
				corners.push_back(cv::Mat(2, cur_fr->corners.size(), CV_32F));
				for (int i = 0; i < cur_fr->corners[0].size(); i++) {
					corners.back().at<float>(X, i) = cur_fr->corners[0][i].x;
					corners.back().at<float>(Y, i) = cur_fr->corners[0][i].y;
				}*/

				base_fr->corners.push_back(cur_fr->corners[0]);

				double err = cv::calibrateCamera(g_obj_points, base_fr->corners, cv::Size(base_fr->img.cols, base_fr->img.rows), camera[0], dist[0], rots, transes);
#endif // #if STEREO_CALIBRATE

				std::cout << "Reprojection error : " << err << std::endl;

				std::cout << "dist[0] : " << dist[0][0] << " " << dist[0][1] << std::endl;
				std::cout << "camera[0] : " << camera[0] << std::endl;
				std::cout << "camera[1] : " << camera[1] << std::endl;

				break;

                cv::Mat rot_;
                cv::Rodrigues(rot, rot_);

                double angle = cv::norm(rot_);

                if (angle < ANGLE_THRES) {
                    std::cout << "Angle: " << angle << std::endl;

                    std::cout << "Trans: " << rot*trans << std::endl;

                    break;
                }

                std::vector<cv::Point2f> rotated;

                rotated.reserve(len);

                for (int x=m_settings.n_chessboard_cols-2; x>=0; x--)
                    for (int y=0; y<m_settings.n_chessboard_rows-1; y++)
                        rotated.push_back(cur_fr->corners[0][y*(m_settings.n_chessboard_cols-1) + x]);

                std::cout << "rotate" << std::endl;

                cur_fr->corners[0].swap(rotated);
            }
			/*
			cv::Mat img_undistorted;

			cv::undistort(base_fr->img, img_undistorted, camera[0], dist[0]);
			img_undistorted.copyTo(base_fr->img);
			cv::undistort(cur_fr->img, img_undistorted, camera[1], dist[1]);
			img_undistorted.copyTo(cur_fr->img);
			cv::imwrite("points_base.jpg", base_fr->img);
			*/
			cv::Mat R[2], P[2], Q;

//			cv::stereoRectify(camera[0], dist[0], camera[1], dist[1], cv::Size(base_fr->img.rows, base_fr->img.cols),
//				rot, trans, R[0], R[1], P[0], P[1], Q);

			P[0].create(3, 4, CV_64F);
			P[1].create(3, 4, CV_64F);
#if STEREO_CALIBRATE
//			P[0].colRange(0, 3) = camera[0] * cv::Mat::eye(3, 3, CV_64F);
//			P[0].colRange(3, 4) = cv::Mat::zeros(3, 1, CV_64F);

//			P[1].colRange(0, 3) = camera[1] * rot;
//			P[1].colRange(3, 4) = camera[1] * trans;
#else
			cv::Mat rot;

			cv::Rodrigues(rots[0], rot);
			P[0].colRange(0, 3) = camera[0] * rot;
			P[0].colRange(3, 4) = camera[0] * transes[0];

			cv::Rodrigues(rots[1], rot); 
			P[1].colRange(0, 3) = camera[0] * rot;
			P[1].colRange(3, 4) = camera[0] * transes[1];

#endif // #if STEREO_CALIBRATE

			std::cout << "proj mat0: " << P[0] << std::endl;
			std::cout << "proj mat1: " << P[1] << std::endl;

            if (tries >= 4) {
                std::cout << "calibration failed" << std::endl;
			} else {
				cur_fr->diff[base_fr->idx].rot   = rot;
				cur_fr->diff[base_fr->idx].trans = trans;

//				draw_camera(trans, rot, cur_fr->idx);

				for (int i = 0; i < base_fr->corners.front().size(); i++) {
					g_projPt_base.at<float>(X, i) = base_fr->corners.front()[i].x;
					g_projPt_base.at<float>(Y, i) = base_fr->corners.front()[i].y;

					g_projPt_cur.at<float>(X, i) = cur_fr->corners.front()[i].x;
					g_projPt_cur.at<float>(Y, i) = cur_fr->corners.front()[i].y;
				}

				recon_scene(m_mesh_fstr, trans, rot, base_fr->img, cur_fr->img, P[0], P[1]);
            }
        }

        break;
    }

    close();
}

static int read_images(std::vector<cv::Mat>& images, const std::string& fmask)
{
	std::vector<boost::filesystem::path> fnames;
	std::vector<std::string> fimages;

	std::copy(boost::filesystem::directory_iterator("."), boost::filesystem::directory_iterator(), std::back_inserter(fnames));

	boost::regex e(fmask);

	foreach_(boost::filesystem::path& fname, fnames)
		if (boost::regex_search(fname.generic_string(), e))
			fimages.push_back(fname.generic_string()); // TODO: There is simpler way, i believe.

	std::sort(fimages.begin(), fimages.end());

	foreach_(const std::string& fimage, fimages)
		images.push_back(cv::imread(fimage));

	return images.size();
}

void IScanner::Impl::open()
{
	m_frm_cnt = read_images(m_images, m_settings.s_images_fmask);

	m_mesh_fstr.open(m_settings.s_mesh_fname);
	m_trajectory_fstr.open(m_settings.s_trajectory_fname);

    const CvSize fr_size = cvSize(m_images[0].rows, m_images[0].cols);

    m_recon.open(m_settings.s_recon_fname, CV_FOURCC('Y','U','Y','2'), 30, fr_size, true);

    if (!m_recon.isOpened())
        throw Exception("Recon cannot be opened. Check if you have opencv_ffmpeg247.dll");
}

void IScanner::Impl::close()
{
    m_recon.release();
    m_trajectory_fstr.close();
	m_mesh_fstr.close();
}

static void draw_cube(std::ofstream& fstr, const cv::Mat& trans, const cv::Mat& rot, int n)
{
	const float d = 0.01;

	const int red = n*4 % 256;

	fstr << "v " << trans.at<double>(X) - d << " " << trans.at<double>(Y) - d << " " << trans.at<double>(Z) - d << " " << red << " 0 0" << std::endl; // -8
	fstr << "v " << trans.at<double>(X) + d << " " << trans.at<double>(Y) - d << " " << trans.at<double>(Z) - d << " " << red << " 0 0" << std::endl; // -7
	fstr << "v " << trans.at<double>(X) + d << " " << trans.at<double>(Y) + d << " " << trans.at<double>(Z) - d << " " << red << " 0 0" << std::endl; // -6
	fstr << "v " << trans.at<double>(X) - d << " " << trans.at<double>(Y) + d << " " << trans.at<double>(Z) - d << " " << red << " 0 0" << std::endl; // -5

	fstr << "v " << trans.at<double>(X) - d << " " << trans.at<double>(Y) - d << " " << trans.at<double>(Z) + d << " " << red << " 0 0" << std::endl; // -4
	fstr << "v " << trans.at<double>(X) + d << " " << trans.at<double>(Y) - d << " " << trans.at<double>(Z) + d << " " << red << " 0 0" << std::endl; // -3
	fstr << "v " << trans.at<double>(X) + d << " " << trans.at<double>(Y) + d << " " << trans.at<double>(Z) + d << " " << red << " 0 0" << std::endl; // -2
	fstr << "v " << trans.at<double>(X) - d << " " << trans.at<double>(Y) + d << " " << trans.at<double>(Z) + d << " " << red << " 0 0" << std::endl; // -1

	fstr << "f -8 -5 -6 -7" << std::endl;
	fstr << "f -4 -3 -2 -1" << std::endl;

	fstr << "f -8 -7 -3 -4" << std::endl;
	fstr << "f -6 -5 -1 -2" << std::endl;

	fstr << "f -5 -8 -4 -1" << std::endl;
	fstr << "f -7 -6 -2 -3" << std::endl;
}

void IScanner::Impl::draw_camera(const cv::Mat& trans, const cv::Mat& rot, int fr_idx)
{
	draw_cube(m_trajectory_fstr, trans, rot, fr_idx);
}
