#pragma once

#include <observation_tree/CObservationTreeModel.h>

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CLineMatching
{
public:
	CLineMatching(CObservationTreeModel *model);
	~CLineMatching();
	void extractLines();
	void runSegmentation();

private:
	CObservationTreeModel *m_model;
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>> m_extracted_lines;
};
