#include "recon.h"

Recon::Recon()
{
}

Recon::~Recon()
{
}

void Recon::clear()
{
	points.clear();
	cameras.clear();
}

void Recon::add(const Camera &cam, const vector<Mat> &images)
{
}