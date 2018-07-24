#pragma once

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
	 * \param model the synced rawlog model.
	 * \param sync_obs_indices indices of the grouped (synchronized) observations in the original model, per sensor.
	 * \param params parameters for the algorithm.
	 */
	CCalibFromPlanesGui(CObservationTreeGui *model, TCalibFromPlanesParams *params);

	~CCalibFromPlanesGui();

	void run();

	/** Runs plane segmentation. */
	void extractPlanes();

	/** Runs plane matching. */
	void matchPlanes();

	/** Adds observer to list of text observers. */
	void addTextObserver(CTextObserver *observer);

	/** Adds observer to list of plane observers. */
	void addPlanesObserver(CPlanesObserver *observer);

	/** Notifies observers with a text message. */
	void publishText(const std::string &msg);

	/** Notifies observers with the extracted planes.
	 * \param sensor_id id of the sensor the planes were observed from.
	 * \param sync_obs_id id of the observation in the synchronized rawlog.
	 */
	void publishPlanes(const int &sensor_id, const int &sync_obs_id);

	/** Notifies observers with the planes of a sensor that were matched in a set, if any.
	 * \param obs_set_id the id of the synchronized observation set.
	 * \param sensor_id id of the sensor the planes were observed from.
	 */
	void publishMatches(const int &obs_set_id, const int &sensor_id);

	/** Returns the status of the calibration progress. */
	CalibrationStatus calibStatus();

private:

	/** The parameters for the calibration. */
	TCalibFromPlanesParams *m_params;

	/** Pointer to the received synchronized rawlog model. */
	CObservationTreeGui *m_sync_model;

	/** List of observers to be notified about progress status. */
	std::vector<CTextObserver*> m_text_observers;

	/** List of observers to be notified about extracted planes. */
	std::vector<CPlanesObserver*> m_planes_observers;
};
