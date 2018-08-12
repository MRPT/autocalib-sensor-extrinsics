#pragma once

#include <observation_tree/CObservationTreeGui.h>
#include <interfaces/CTextObserver.h>
#include <interfaces/CPlanesObserver.h>
#include <interfaces/CCorrespPlanesObserver.h>
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
	 * \param params parameters for the algorithm.
	 */
	CCalibFromPlanesGui(CObservationTreeGui *model, TCalibFromPlanesParams *params);

	~CCalibFromPlanesGui();

	void run();

	/** Runs plane segmentation. */
	void extractPlanes();

	/** Runs plane matching. */
	void matchPlanes();

	/** Runs the calibration solver. */
	void calibrate();

	/** Adds observer to list of text observers. */
	void addTextObserver(CTextObserver *observer);

	/** Adds observer to list of plane observers. */
	void addPlanesObserver(CPlanesObserver *observer);

	/** Adds observer to list of matched planes observers. */
	void addCorrespPlanesObserver(CCorrespPlanesObserver *observer);

	/** Notifies text observers with a message. */
	void publishText(const std::string &msg);

	/** Notifies observers with the extracted planes.
	 * \param sensor_id id of the sensor whose observervation the planes were extracted from.
	 * \param sync_obs_id id of the observation in the synchronized rawlog.
	 */
	void publishPlanes(const int &sensor_id, const int &sync_obs_id);

	/** Notifies observers with the planes that were matched between each pair of sensors in a sync set, if any.
	 * \param obs_set_id the id of the synchronized observation set.
	 */
	void publishCorrespPlanes(const int &obs_set_id);

	/** Returns the status of the calibration progress. */
	CalibFromPlanesStatus calibStatus();

private:

	/** List of text observers to be notified about the progress. */
	std::vector<CTextObserver*> m_text_observers;

	/** List of observers to be notified about the extracted planes. */
	std::vector<CPlanesObserver*> m_planes_observers;

	/** List of observers to be notified about the matched planes. */
	std::vector<CCorrespPlanesObserver*> m_corresp_planes_observers;
};
