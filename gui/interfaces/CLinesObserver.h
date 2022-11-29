#pragma once

#include <CLine.h>

/**
 * \brief Observer (listener) that receives extracted lines for visualization from the GUI core wrappers.
 */

class CLinesObserver
{
public:
	virtual void onReceivingLines(const int &viewer_id, const std::vector<CLine> &lines) = 0;
};
