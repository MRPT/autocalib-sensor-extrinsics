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
	void addTextObserver(CTextObserver *observer);
	void addPlanesObserver(CPlanesObserver *observer);
	void publishText(const std::string &msg);
	void publishPlaneCloud(const int &set_num /* the id of the set the plane cloud belongs to */,
						   const int &cloud_num /* the id of the plane cloud inside the set */,
						   const int &sensor_id /* the id of the sensor the planes belong to */);

private:
	CObservationTreeModel *m_model;
	std::array<double,6> m_init_calib;
	TPlaneMatchingParams m_params;
	std::vector<CTextObserver*> m_text_observers;
	std::vector<CPlanesObserver*> m_planes_observers;
	//The extracted plane clouds are stored in the same format as the observation tree
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>> m_extracted_plane_clouds_sets;
};
