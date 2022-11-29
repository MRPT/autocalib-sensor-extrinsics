#pragma once

#include <CPlane.h>
#include <CLine.h>
#include <mrpt/img/CImage.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/** Stores data and features from a sensor's observation for display in the viewers later. */

struct TViewerBuffer
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
	std::vector<CPlaneCHull> planes;
	std::vector<CLine> lines;
	mrpt::img::CImage::Ptr image;
	bool draw;
	std::string text;
	std::string set_text;
};
