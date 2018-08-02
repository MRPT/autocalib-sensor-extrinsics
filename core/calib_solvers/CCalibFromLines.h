#pragma once

#include "CExtrinsicCalib.h"
#include "TCalibFromLinesParams.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/common/time.h>

/**
 * \brief Exploit 3D line observations from a set of sensors to perform extrinsic calibration.
 */

class CCalibFromLines : public CExtrinsicCalib
{
public:

	/** The segmented lines, the vector indices to access them are [sensor_id][obs_id][line_id]
	 * obs_id is with respect to the synchronized model.
	 */
	std::map<int,std::vector<std::vector<cv::Vec4i>>> mvv_lines;

	/** The line correspondences between the different sensors.
	 * The map indices correspond to the sensor ids, with the list of correspondeces
	 * stored as a matrix with each row of the form - set_id, line_id1, line_id2.
	 */
	std::map<int,std::map<int,std::vector<std::array<int,3>>>> mmv_line_corresp;

	CCalibFromLines(CObservationTree *model);
	~CCalibFromLines();

	/**
	 * \brief Runs Canny-Hough, and Bresenham algorithm on a single image.
	 * \param image the input image
	 * \param lines vector of lines segmented
	 */
	void segmentLines(const cv::Mat &image, const TLineSegmentationParams &params, std::vector<cv::Vec4i> &lines);

	/** Calculate the angular residual error of the correspondences.
	 * \param sensor_poses relative poses of the sensors
	 * \return the residual
	 */
	virtual Scalar computeRotCalibResidual(const std::vector<Eigen::Matrix4f> &sensor_poses);

	/** Compute Calibration (only rotation).
	 * \param sensor_poses initial calibration
	 * \return the residual
	 */
	virtual Scalar computeRotCalibration(const TSolverParams &params, const std::vector<Eigen::Matrix4f> &sensor_poses, std::string &stats);
};
