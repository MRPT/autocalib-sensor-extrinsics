#pragma once

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CObserver
{
public:
	virtual void onReceivingText(const std::string &msg) = 0;
	virtual void onReceivingPlaneCloud(const int &sensor_id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) = 0;
};
