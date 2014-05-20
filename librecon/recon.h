#pragma once
#include <vector>
#include <opencv2/core/core.hpp>

#include "camera.h"

using namespace std;
using namespace cv;

class Recon
{
public:
	Recon();
	~Recon();

	void clear();
	void add(const Camera &camera, const vector<Mat> &images);
	
    vector<vector<Point3d> > get_points() const;
	vector<Camera> get_cameras() const;
private:
    vector<vector<Point3d> > points;
	vector<Camera> cameras;
};
