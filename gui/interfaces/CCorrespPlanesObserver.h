#pragma once

#include <CPlanes.h>

/**
 * \brief Observer (listener) that receives the matched planes of a set for visualization from the GUI core wrappers.
 */

class CCorrespPlanesObserver
{
public:
	virtual void onReceivingCorrespPlanes(const std::vector<std::vector<CPlaneCHull>> &corresp_planes) = 0;
};
