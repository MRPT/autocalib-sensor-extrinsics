#pragma once

#include <iostream>

class CSubscriber
{
public:
	virtual void onEvent(const std::string &msg) = 0;
};
