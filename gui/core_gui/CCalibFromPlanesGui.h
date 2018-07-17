#pragma once

#include "TPlaneMatchingParams.h"
#include <observation_tree/CObservationTreeGui.h>
#include <interfaces/CTextObserver.h>
#include <interfaces/CPlanesObserver.h>
#include <calib_solvers/CCalibFromPlanes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <functional>

/**
 * Provides a GUI wrapper around the core calibration from planes classes.
 *
 * Inherits from CCalibFromPlanes.
 */
class CCalibFromPlanesGui : public CCalibFromPlanes
{
public:

	/**
	 * Constructor
	 * \param model the rawlog model.
	 * \param sync_obs_indices indices of the grouped (synchronized) observations in the original model, per sensor.
	 * \param params parameters for the algorithm.
	 */
	CCalibFromPlanesGui(CObservationTreeGui *model, TPlaneMatchingParams params);

	~CCalibFromPlanesGui();

	void run();

	void proceed();

	/** Runs plane segmentation. */
	void extractPlanes();

	/** Adds observer to list of text observers. */
	void addTextObserver(CTextObserver *observer);

	/** Adds observer to list of plane observers. */
	void addPlanesObserver(CPlanesObserver *observer);

	/** Notifies observers with text message. */
	void publishText(const std::string &msg);

	/** Notifies observers with the extracted plane cloud.
	 * \param sensor_id the id of the sensor the plane cloud belongs to.
	 * \param obs_id id of the observation in the original rawlog.
	 */
	void publishPlaneCloud(const int &sensor_id, int obs_id);

private:

	/** The parameters for the calibration. */
	TPlaneMatchingParams m_params;

	/** Pointer to the received synchronized rawlog model. */
	CObservationTreeGui *m_sync_model;

	/** List of observers to be notified about progress status. */
	std::vector<CTextObserver*> m_text_observers;

	/** List of observers to be notified about extracted planes. */
	std::vector<CPlanesObserver*> m_planes_observers;
};
