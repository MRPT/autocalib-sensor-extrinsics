#pragma once

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CObserver
{
public:
	virtual void onEvent(const std::string &msg) = 0;
	virtual void onCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) = 0;
};
