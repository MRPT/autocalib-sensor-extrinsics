#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CPlanesObserver
{
public:
    virtual void onReceivingPlaneCloud(const int &sensor_id, const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &cloud) = 0;
};
