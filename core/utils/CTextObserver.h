#pragma once

#include <iostream>

class CTextObserver
{
public:
	virtual void onReceivingText(const std::string &msg) = 0;
};
