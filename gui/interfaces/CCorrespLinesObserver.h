#pragma once

#include <CLine.h>
#include <map>

/**
 * \brief Observer (listener) that receives the matched lines between each sensor pair in an observation set for visualization.
 */

class CCorrespLinesObserver
{
public:
	virtual void onReceivingCorrespLines(std::map<int,std::map<int,std::vector<std::array<CLine,2>>>> &corresp_lines, const std::vector<Eigen::Matrix4f> &sensor_poses) = 0;
};
