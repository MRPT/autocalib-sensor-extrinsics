/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CMatrix.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef float Scalar;

/** Store the plane extracted from a depth image (or point cloud) defined by some geometric characteristics. */
class CPlane
{
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<Scalar,3,1> v3center;
    Eigen::Matrix<Scalar,3,1> v3normal;
    Scalar d;
};

/** Store the plane's geometric characteristics and its convex hull. */
class CPlaneCHull : public CPlane
{
  public:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ConvexHullPtr;
    std::vector<size_t> v_hull_indices;
    std::vector<int> v_inliers;
};
