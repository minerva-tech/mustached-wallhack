#ifndef SCANNER_H
#define SCANNER_H

class IScanner::Impl {
public:
	Impl(const Settings& settings);

	void process();

private:
    struct FrameData;
    struct Diff;

	std::vector<cv::Mat> m_images;

//	cv::VideoCapture m_capture;
    cv::VideoWriter  m_recon;

	void open();
	void close();

    void draw_camera(const cv::Mat& trans, const cv::Mat& rot, int fr_idx);

	struct Camera;

    Settings m_settings;

    const std::vector<std::vector<cv::Point3f> > g_obj_points;

    std::vector<FrameData> m_frames;

	std::ofstream m_mesh_fstr;
	std::ofstream m_trajectory_fstr;

	cv::Mat m_sum_rot;

    int m_frm_cnt;

//	bool calibrate_camera(Camera& cam);

//	cv::Mat write_camera_pos(const Camera& cam, const cv::Mat& cur_pos);
};

struct IScanner::Impl::Diff
{
    cv::Mat rot;
    cv::Mat trans;
};

struct IScanner::Impl::FrameData
{
    uint32_t idx;

    std::vector<std::vector<cv::Point2f> > corners;

    std::map<uint32_t, Diff> diff;
};

struct IScanner::Impl::Camera
{
	Camera() {
//		rot.create(3, 1, CV_64F);
//		trans.create(3, 1, CV_64F);
	}

	cv::Mat camera[2];
	std::vector<float> dist[2];
	cv::Mat rot;
	cv::Mat trans;
	cv::Mat E;
	cv::Mat F;
};

#endif
