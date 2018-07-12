#pragma once

#include <iostream>

/**
 * @brief Observer (listener) that receives text updates from the GUI core wrappers.
 */
class CTextObserver
{
public:
	virtual void onReceivingText(const std::string &msg) = 0;
};
