
#include "pchbarrier.h"
#include "settings.h"

Settings::Settings() :
	n_feature_points	(100),
	f_cube_size			(0.1),
	b_trajectory_points	(true),
	n_chessboard_rows	(10),
	n_chessboard_cols	(7),
	n_begin_frame		(0),
	n_end_frame			(50),
	s_images_fmask		("IMG*"),
    s_recon_fname       ("./recon.yuv"),
	s_mesh_fname		("./output.obj"),
	s_trajectory_fname	("./trajectory.obj")
{
}
