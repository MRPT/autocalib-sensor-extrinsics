#pragma once

#include <CPlane.h>
#include <map>

/**
 * \brief Observer (listener) that receives the matched planes between each sensor pair in an observation set for visualization.
 */

class CCorrespPlanesObserver
{
public:
	virtual void onReceivingCorrespPlanes(std::map<int,std::map<int,std::vector<std::array<CPlaneCHull,2>>>> &corresp_planes, const std::vector<Eigen::Matrix4f> &sensor_poses) = 0;
};
