#pragma once

#include <calib_solvers/TPlaneMatchingParams.h>
#include <observation_tree/CObservationTreeModel.h>
#include <utils/CObserver.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <functional>

//Class definition for the plane matching algorithm

class CPlaneMatching
{
public:
	CPlaneMatching(CObservationTreeModel *model, std::array<double,6> init_calib, TPlaneMatchingParams params);
	~CPlaneMatching();
	void run();
	void proceed();
	void extractPlanes();
	void runSegmentation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &extracted_planes);
	void addObserver(CObserver *observer);
	void publishEvent(const std::string &msg);
	void publishPlaneCloud(const int &set_num /* the id of the set the plane cloud belongs to */,
						   const int &cloud_num /* the id of the plane cloud inside the set */,
						   const int &sensor_id /* the id of the sensor the planes belong to */);

private:
	CObservationTreeModel *m_model;
	std::array<double,6> m_init_calib;
	TPlaneMatchingParams m_params;
	std::vector<CObserver*> m_observers;
	//The extracted plane clouds are stored in the same format as the observation tree
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>> m_extracted_plane_clouds_sets;
};
