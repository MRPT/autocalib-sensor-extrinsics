#pragma once

#include "CExtrinsicCalib.h"
#include "TCalibFromLinesParams.h"
#include "CLine.h"

#include <mrpt/img/TCamera.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/common/time.h>

/**
 * \brief Exploit 3D line observations from a set of sensors to perform extrinsic calibration.
 */

class CCalibFromLines : public CExtrinsicCalib
{
protected:

	/** The parameters for the calibration. */
	TCalibFromLinesParams *params;

	/** The segmented lines, the vector indices to access them are [sensor_id][obs_id][line_id]
	 * obs_id is with respect to the synchronized model.
	 */
	std::map<int,std::vector<std::vector<CLine>>> mvv_lines;

	/** The line correspondences between the different sensors.
	 * The map indices correspond to the sensor ids, with the list of correspondeces
	 * stored as a matrix with each row of the form - set_id, line_id1, line_id2.
	 */
	std::map<int,std::map<int,std::vector<std::array<int,3>>>> mmv_line_corresp;

	CCalibFromLines(CObservationTree *model, TCalibFromLinesParams *params);
	~CCalibFromLines();

	/**
	 * \brief Runs Canny-Hough, Bresenham algorithm, and subsequent back-projection to extract 2D and 3D lines from RGB-D images.
	 * The 3D lines extracted are in the depth sensor's frame.
	 * \param image the input image
	 * \param lines vector of lines segmented
	 */
	void segmentLines(const cv::Mat &image, Eigen::MatrixXf &range, const mrpt::img::TCamera &camera_params, const Eigen::Affine3f &intensity_to_depth_transform, std::vector<CLine> &lines);

	/**
	 * Search for potential line matches between each sensor pair in a syc obs set.
	 * \param lines lines extracted from sensor observations that belong to the same synchronized set. lines[sensor_id][line_id] gives a line.
	 * \param set_id the id of the synchronized set the lines belong to.
	 * \param params the parameters for line matching.
	 */
	void findPotentialMatches(const std::vector<std::vector<CLine>> &lines, const int &set_id);

	/** Calculate the angular residual error of the correspondences.
	 * \return the residual
	 */
	virtual Scalar computeRotationResidual();

	/** Compute Calibration (only rotation).
	 * \return the residual
	 */
	virtual Scalar computeRotation();

    /** Compute Calibration (only translation).
        \return the residual */
	virtual Scalar computeTranslation();
};
