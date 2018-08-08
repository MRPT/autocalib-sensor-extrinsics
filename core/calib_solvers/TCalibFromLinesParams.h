#pragma once

#include "TExtrinsicCalibParams.h"
#include <vector>
#include <Eigen/Dense>

struct TLineSegmentationParams
{
	//params for canny edge detection
	int clow_threshold;
	int chigh_to_low_ratio;
	int ckernel_size;

	//params for hough transform
	int hthreshold;
};

struct TLineMatchingParams
{
	double min_normals_dot_prod;
	double max_line_normal_dot_prod;
};

/**
 * Maintains the status of the calibration progress.
 * This is useful when the calibration is run in steps and
 * the results of each step are to be visualized.
 */
enum CalibrationFromLinesStatus
{
	LCALIB_YET_TO_START,
	LINES_EXTRACTED,
	LINES_MATCHED
};

struct TCalibFromLinesParams
{
	int downsample_factor;
	TLineSegmentationParams seg;
	TLineMatchingParams match;
	TSolverParams solver;
	CalibrationFromLinesStatus calib_status;
};
