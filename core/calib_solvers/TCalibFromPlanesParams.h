#pragma once

#include <vector>
#include <Eigen/Core>

struct TPlaneSegmentationParams
{
	//params for integral normal estimation
	int normal_estimation_method;
	bool depth_dependent_smoothing;
	double max_depth_change_factor;
	double normal_smoothing_size;

	//params for organized multiplane segmentation
	double angle_threshold;
	double dist_threshold;
	double min_inliers_frac;
	double max_curvature;
};

struct TPlaneMatchingParams
{
	double normals_dot_prod;
	double dist_diff;
};

struct TCalibFromPlanesParams
{
	TPlaneSegmentationParams seg;
	TPlaneMatchingParams match;
};
