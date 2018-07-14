#pragma once

#include <observation_tree/CObservationTreeModel.h>

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * Provides a GUI wrapper around the core calibration from lines classes.
 */

class CCalibFromLinesGui
{
public:

	/**
	 * Constructor
	 * \param model the rawlog model.
	 * \param sync_obs_indices sync_obs_indices indices of the grouped (synchronized) observations in the original model, per sensor.
	 */
	CCalibFromLinesGui(CObservationTreeModel *model, std::vector<std::vector<int>> &sync_obs_indices);

	~CCalibFromLinesGui();

	/** Extracts lines from from all the image observation in the model. */
	void extractLines();

	/** Runs Canny-Hough, and Bresenham algorithm to return a vector of segments
	 * from a single image.*/
	std::vector<cv::Vec4i> runSegmentation(const cv::Mat &image);

private:

	/** Pointer to the rawlog tree model */
	CObservationTreeModel *m_model;
};
