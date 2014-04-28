#include "pchbarrier.h"
#include "settings.h"
#include "defs.h"
#include "iscanner.h"
#include "scanner.h"
#include "exception.h"

#ifdef _DEBUG
#pragma comment(lib, "opencv_core247d.lib")
#pragma comment(lib, "opencv_highgui247d.lib")
#pragma comment(lib, "opencv_flann247d.lib")
#pragma comment(lib, "opencv_imgproc247d.lib")
#pragma comment(lib, "opencv_features2d247d.lib")
#pragma comment(lib, "opencv_calib3d247d.lib")
#pragma comment(lib, "zlibd.lib")
#pragma comment(lib, "libtiffd.lib")
#pragma comment(lib, "libjpegd.lib")
#pragma comment(lib, "libpngd.lib")
#pragma comment(lib, "libjasperd.lib")
#pragma comment(lib, "IlmImfd.lib")
#pragma comment(lib, "vfw32.lib")
#else
#pragma comment(lib, "opencv_core247.lib")
#pragma comment(lib, "opencv_highgui247.lib")
#pragma comment(lib, "opencv_calib3d247.lib")
#endif

static std::vector<std::vector<cv::Point3f> > fill_obj_points(int w, int h)
{
	std::vector<std::vector<cv::Point3f> > ms;

	ms.push_back(std::vector<cv::Point3f>());

	std::vector<cv::Point3f>& m = ms.back();

	m.resize((w-1)*(h-1));

	const double x_step = 1./(double)w;
	const double y_step = 1./(double)h;

	for(uint32_t i=0; i<m.size(); i++) {
		const double x = i % (w-1);
		const double y = i / (h-1);

		m[i].x = (float)(x * x_step);
		m[i].y = (float)(y * y_step);
		m[i].z = 0.;
	}

	return ms;
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

	cv::Mat pos = cv::Mat::zeros(3, 1, CV_64F);

    cv::Mat img;

    std::vector <cv::Point2f> corners;

    corners.resize((m_settings.n_chessboard_cols-1) * (m_settings.n_chessboard_rows-1));

    const CvSize pat_size = cvSize(m_settings.n_chessboard_cols-1, m_settings.n_chessboard_rows-1);

    int fr_cnt = 0;

    while (m_capture.read(img) && fr_cnt < m_settings.n_end_frame) {
	    const bool found_fast = findChessboardCorners(img, pat_size, corners, cv::CALIB_CB_FAST_CHECK);

        if (found_fast) {
            const bool found = findChessboardCorners(img, pat_size, corners);

            if (found) {
                drawChessboardCorners(img, pat_size, corners, found);

                m_recon.write(img);

                m_frames.push_back(FrameData());

                m_frames.back().corners.push_back(corners);
                m_frames.back().idx = fr_cnt;

                std::cout << "frame #" << fr_cnt << " chessboard found." << std::endl;
            } else {
                std::cout << "frame #" << fr_cnt << " chessboard NOT found." << std::endl;
            }
        } else {
            std::cout << "frame #" << fr_cnt << " chessboard NOT found." << std::endl;
        }

        fr_cnt++;
    }

    cv::Mat camera[2];
    std::vector<float> dist[2];

    dist[0].resize(9);
    dist[1].resize(9);

   	cv::Mat rot;
	cv::Mat trans;
	cv::Mat E;
	cv::Mat F;

    const int len = (m_settings.n_chessboard_cols-1) * (m_settings.n_chessboard_rows-1);

    const double ANGLE_THRES = CV_PI / 4.; // if angle between two camera positions is bigger than this, chessboard should be rotated. I suppose that there is no large jumps in video sequence. All neighbor frames are close each other.

    for(auto base_fr = m_frames.begin(); base_fr+1 != m_frames.end(); ++base_fr) {
        for(auto cur_fr = base_fr+1; cur_fr != m_frames.end(); ++cur_fr) {
            int tries = 0;
            while (tries++ < 4) {
                cv::stereoCalibrate(g_obj_points, base_fr->corners, cur_fr->corners, camera[0], dist[0], camera[1], dist[1], cvSize(1, len), rot, trans, E, F, 
    		        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 1e-6), CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);

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

                cur_fr->corners[0] = rotated;
            }

            if (tries >= 4) {
                std::cout << "calibration failed" << std::endl;
            } else {
                cur_fr->diff[base_fr->idx].rot   = rot;
                cur_fr->diff[base_fr->idx].trans = trans;

                draw_camera(trans, rot, cur_fr->idx);
            }
        }

        break;
    }

    close();
}

void IScanner::Impl::open()
{
	m_capture.open(m_settings.s_video_fname);

	if (!m_capture.isOpened())
		throw Exception("Video cannot be opened. Check if you have opencv_ffmpeg247.dll");

	m_mesh_fstr.open(m_settings.s_mesh_fname);
	m_trajectory_fstr.open(m_settings.s_trajectory_fname);

	m_frm_cnt = static_cast<int>(m_capture.get(CV_CAP_PROP_FRAME_COUNT));
    
    const int w = static_cast<int>(m_capture.get(CV_CAP_PROP_FRAME_WIDTH));
    const int h = static_cast<int>(m_capture.get(CV_CAP_PROP_FRAME_HEIGHT));

    std::cout << "resolution: " << w << "x" << h << std::endl; 

    const CvSize fr_size = cvSize(w, h);

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

/*
bool IScanner::Impl::calibrate_camera(Camera& cam)
{
	std::vector<std::vector <cv::Point2f> > corners[2];
	std::vector <cv::Point2f>* corner[2];

	for (int i=0; i<2; i++) {
		corners[i].push_back(std::vector <cv::Point2f>());

		corner[i] = &corners[i].back();

		// bug in C++ wrapper around findChessboardCornders? anyway, todo: check it. But without this resize find_movement method crashes on exit.
		corner[i]->resize((m_settings.n_chessboard_cols-1) * (m_settings.n_chessboard_rows-1));
	}

	const bool found_fast_0 = findChessboardCorners(frame(0), cvSize(m_settings.n_chessboard_cols-1, m_settings.n_chessboard_rows-1), 
		*corner[0], cv::CALIB_CB_FAST_CHECK);
	const bool found_fast_1 = findChessboardCorners(frame(1), cvSize(m_settings.n_chessboard_cols-1, m_settings.n_chessboard_rows-1), 
		*corner[1], cv::CALIB_CB_FAST_CHECK);

	if (!found_fast_0 && found_fast_1) {
		freeze(true);
		return false;
	}

	if (!found_fast_0 || !found_fast_1)
		return false;

	freeze(false);

	const bool found_0 = findChessboardCorners(frame(0), cvSize(m_settings.n_chessboard_cols-1, m_settings.n_chessboard_rows-1), *corner[0]);
	const bool found_1 = findChessboardCorners(frame(1), cvSize(m_settings.n_chessboard_cols-1, m_settings.n_chessboard_rows-1), *corner[1]);

	if (!found_0 || !found_1)
		return false;

	const int len = (m_settings.n_chessboard_cols-1) * (m_settings.n_chessboard_rows-1);

//	cv::calibrateCamera(g_obj_points, corners, cvSize(1, len), cam.camera, cam.dist, cam.rot, cam.trans);

	cam.dist[0].resize(9);
	cam.dist[1].resize(9);

	cv::stereoCalibrate(g_obj_points, corners[1], corners[0], cam.camera[1], cam.dist[1], cam.camera[0], cam.dist[0], 
		cvSize(1, len), cam.rot, cam.trans, cam.E, cam.F, 
		cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6), CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);

//	cam.rot = cv::Mat::ones(3, 3, CV_64F);
//	cam.trans = cv::Mat::ones(3, 1, CV_64F);

	return true;
}

void IScanner::Impl::write_aff_mat(const AffMatrix& mat) const
{
	std::cout << mat.mat << std::endl;
}

void IScanner::Impl::write_points(const Points& pts)
{

}
*/

static void draw_cube(std::ofstream& fstr, const cv::Mat& trans, const cv::Mat& rot, int n)
{
	const float d = 0.1;

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

/*
cv::Mat IScanner::Impl::write_camera_pos(const Camera& cam, const cv::Mat& cur_pos)
{
//	std::cout << cam.rot << std::endl;
	std::cout << cam.trans << std::endl;
	std::cout << cam.rot << std::endl;

//	const double theta = cv::norm(cam.rot[0]);
//	cam.rot[0] /= theta;

//	and so on

//	cv::Mat rot;

//	cv::Rodrigues(cam.rot, rot);

//	draw_cube(m_trajectory_fstr, cam.rot*cam.trans, cam.rot);

	m_sum_rot *= cam.rot;
	const auto new_pos = cur_pos + m_sum_rot*cam.trans;
//	const auto new_pos = cur_pos + cam.trans;
//	const auto new_pos = cur_pos + cam.rot*cam.trans;

	const auto delta = m_sum_rot*cam.trans;

	std::cout << "Delta: " << delta << std::endl;

	draw_cube(m_trajectory_fstr, new_pos, cam.rot, m_frm_processed);

	return new_pos;
}
*/