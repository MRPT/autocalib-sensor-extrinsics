#pragma once

#include "TPlaneMatchingParams.h"
#include <observation_tree/CObservationTreeModel.h>
#include <interfaces/CTextObserver.h>
#include <interfaces/CPlanesObserver.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <functional>

/**
 * Provides a GUI wrapper around the core calibration from planes classes.
 */
class CCalibFromPlanesGui
{
public:
	CCalibFromPlanesGui(CObservationTreeModel *model, std::array<double,6> init_calib, TPlaneMatchingParams params);
	~CCalibFromPlanesGui();
	void run();
	void proceed();
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
	CObservationTreeModel *m_model;
	std::array<double,6> m_init_calib;
	TPlaneMatchingParams m_params;

	/** List of observers to be notified about progress status. */
	std::vector<CTextObserver*> m_text_observers;

	/** List of observers to be notified about extracted planes. */
	std::vector<CPlanesObserver*> m_planes_observers;

	/** The extracted plane clouds are stored in the same format as the observation tree. */
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>> m_extracted_plane_clouds_sets;
};
