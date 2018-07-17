#pragma once

#include <iostream>
#include<vector>

/** Function template to find the position of an element in a vector. */

template <typename T>
size_t findItemIndexIn(const std::vector<T> &vec, const T &item)
{
	auto iter = std::find(vec.begin(), vec.end(), item);
	return std::distance(vec.begin(), iter);
}
