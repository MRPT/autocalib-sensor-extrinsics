#pragma once

#include <calib_solvers/CPlaneMatchingParams.h>
#include <observation_tree/CObservationTreeModel.h>
//#include <gui/ui_CMainWindow.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//Class definition for the plane matching algorithm

class CPlaneMatching
{
public:
	CPlaneMatching(CObservationTreeModel *model/*, Ui::CMainWindow* main_ui*/, CPlaneMatchingParams params);
	~CPlaneMatching();
	void run();
	void proceed();
	void detectPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::stringstream &stream);

private:
	CObservationTreeModel *m_model;
	//Ui::CMainWindow *m_main_ui;
	std::set<unsigned> observed_planes;
	CPlaneMatchingParams m_params;
};
