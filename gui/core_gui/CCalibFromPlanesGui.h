#pragma once

#include "TPlaneMatchingParams.h"
#include <observation_tree/CObservationTreeModel.h>
#include <interfaces/CTextObserver.h>
#include <interfaces/CPlanesObserver.h>
#include <calib_solvers/CCalibFromPlanes.h>
#include <utils/CPlanes.h>

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
	CCalibFromPlanesGui(CObservationTreeModel *model, TPlaneMatchingParams params);
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
	 * \param set_num the id of the set the plane cloud belongs to.
	 * \param cloud_num the id (index) of the plane cloud inside the set.
	 * \param sensor_id the id of the sensor the plane cloud belongs to.
	 */
	void publishPlaneCloud(const int &set_num, const int &cloud_num, const int &sensor_id);

private:

	/** The parameters for the calibration. */
	TPlaneMatchingParams m_params;

	/** Pointer to the received (synchronized) rawlog model. */
	CObservationTreeModel *m_model;

	/** List of observers to be notified about progress status. */
	std::vector<CTextObserver*> m_text_observers;

	/** List of observers to be notified about extracted planes. */
	std::vector<CPlanesObserver*> m_planes_observers;
};
