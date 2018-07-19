#pragma once

#include <iostream>
#include<vector>

/** Function template to find the position of an element in a vector. */

// Define some colours to draw bolobs, patches, etc.
namespace viz_colors
{
    static const unsigned char red [10] = {255,   0,   0, 255, 255,   0, 255, 204,   0, 255};
	static const unsigned char grn [10] = {  0, 255,   0, 255,   0, 255, 160,  51, 128, 222};
	static const unsigned char blu [10] = {  0,   0, 255,   0, 255, 255, 0  , 204,   0, 173};
}

template <typename T>
size_t findItemIndexIn(const std::vector<T> &vec, const T &item)
{
	auto iter = std::find(vec.begin(), vec.end(), item);
	return std::distance(vec.begin(), iter);
}
