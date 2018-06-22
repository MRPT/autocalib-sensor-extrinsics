#pragma once

#include <calib_solvers/TPlaneMatchingParams.h>
#include <observation_tree/CObservationTreeModel.h>
#include <utils/CSubscriber.h>

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
	void detectPlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
	void addSubscriber(CSubscriber *subscriber);
	void publishEvent(const std::string &msg);

private:
	CObservationTreeModel *m_model;
	std::array<double,6> m_init_calib;
	TPlaneMatchingParams m_params;
	std::vector<CSubscriber*> m_subscribers;
};
