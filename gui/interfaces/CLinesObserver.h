#pragma once

#include<opencv2/core/core.hpp>

/**
 * \brief Observer (listener) that receives extracted lines for visualization from the GUI core wrappers.
 */

class CLinesObserver
{
public:
	virtual void onReceivingLines(const int &viewer_id, const std::vector<cv::Vec4i> &planes) = 0;
};
