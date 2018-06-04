#pragma once

#include "../gui/observation_tree/CObservationTreeModel.h"
#include "../gui/ui_CMainWindow.h"

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

//Class definition for the plane matching calibration algorithm

class CPlaneMatching
{
public:
	CPlaneMatching(double angle_threshold = 4, double dist_threshold = 0.04, double min_inliers_frac = 0.005);
	~CPlaneMatching();
	void run(CObservationTreeModel *model, Ui::CMainWindow* main_ui);
	void setModel(CObservationTreeModel *model);
	void proceed();
	void detectPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::stringstream &stream);

private:
	CObservationTreeModel *m_model;
	double m_angle_threshold, m_dist_threshold, m_min_inliers_frac;
	std::set<unsigned> observed_planes;
};
