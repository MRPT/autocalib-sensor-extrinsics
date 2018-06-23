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
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> extractPlanes();
	void runSegmentation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
	void addObserver(CObserver *observer);
	void publishEvent(const std::string &msg);
	void publishCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);

private:
	CObservationTreeModel *m_model;
	std::array<double,6> m_init_calib;
	TPlaneMatchingParams m_params;
	std::vector<CObserver*> m_observers;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> m_extracted_plane_clouds;
};
