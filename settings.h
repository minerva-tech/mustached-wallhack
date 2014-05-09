#ifndef CONFIG_H
#define CONFIG_H

struct Settings {
	Settings();

	int		n_feature_points;
	double	f_cube_size;
	bool	b_trajectory_points;

	int		n_chessboard_rows;
	int		n_chessboard_cols;

	int		n_begin_frame;
	int		n_end_frame;

	std::string s_images_fmask;
    std::string s_recon_fname;
	std::string s_mesh_fname;
	std::string s_trajectory_fname;
};

#endif