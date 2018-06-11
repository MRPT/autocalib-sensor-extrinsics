#pragma once

#include <calib_solvers/CPlaneMatchingParams.h>
#include <observation_tree/CObservationTreeModel.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <functional>

//Class definition for the plane matching algorithm

class CPlaneMatching
{
public:
	CPlaneMatching(CObservationTreeModel *model, std::function<void(const std::string&)> updateFunction, std::array<double,6> init_calib, CPlaneMatchingParams params);
	~CPlaneMatching();
	void run();
	void proceed();
	void detectPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::stringstream &stream);

private:
	CObservationTreeModel *m_model;
	std::array<double,6> m_init_calib;
	CPlaneMatchingParams m_params;
	std::set<unsigned> observed_planes;

	std::function<void(const std::string&)> sendTextUpdate;
};
