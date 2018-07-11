#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Observer (listener) that receives extracted planes for visualization from the GUI core wrappers.
 */

class CPlanesObserver
{
public:
    virtual void onReceivingPlaneCloud(const int &sensor_id, const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &cloud) = 0;
};
