#pragma once

#include <CPlanes.h>

/**
 * \brief Observer (listener) that receives extracted planes for visualization from the GUI core wrappers.
 */

class CPlanesObserver
{
public:
	virtual void onReceivingPlanes(const int &viewer_id, const std::vector<CPlaneCHull> &planes) = 0;
};
