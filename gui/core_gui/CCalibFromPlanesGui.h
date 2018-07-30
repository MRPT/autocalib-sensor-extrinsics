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

	/** Runs the calibration solver. */
	void calibrate();

	/** Adds observer to list of text observers. */
	void addTextObserver(CTextObserver *observer);

	/** Adds observer to list of plane observers. */
	void addPlanesObserver(CPlanesObserver *observer);

	/** Adds observer to list of matched planes observers. */
	void addCorrespPlanesObserver(CCorrespPlanesObserver *observer);

	/** Notifies observers with a text message. */
	void publishText(const std::string &msg);

	/** Notifies observers with the extracted planes.
	 * \param sensor_id id of the sensor the planes were observed from.
	 * \param sync_obs_id id of the observation in the synchronized rawlog.
	 */
	void publishPlanes(const int &sensor_id, const int &sync_obs_id);

	/** Notifies observers with the planes that were matched between a pair of sensors in a sync set, if any.
	 * \param obs_set_id the id of the synchronized observation set.
	 * \param the id of the first sensor in the pair.
	 * \param the id of the second sensor in the pair.
	 */
	void publishCorrespPlanes(const int &obs_set_id);

	/** Returns the status of the calibration progress. */
	CalibrationStatus calibStatus();

private:

	/** The parameters for the calibration. */
	TCalibFromPlanesParams *m_params;

	/** List of observers to be notified about progress status. */
	std::vector<CTextObserver*> m_text_observers;

	/** List of observers to be notified about extracted planes. */
	std::vector<CPlanesObserver*> m_planes_observers;

	/** List of observers to be notified about the matched planes. */
	std::vector<CCorrespPlanesObserver*> m_corresp_planes_observers;
};
