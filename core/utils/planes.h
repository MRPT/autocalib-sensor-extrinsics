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

/** Store the plane extracted from a depth image (or point cloud) defined by some geometric characteristics.
 */
//template<typename Scalar>
class Plane
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<Scalar,3,1> v3center;
    Eigen::Matrix<Scalar,3,1> v3normal;
    Scalar d;
    Eigen::Matrix<Scalar,4,4> covariance;  // the inverse of Fisher information matrix
    Scalar curvature;
    Scalar area;
    size_t n_inliers;
};

/** Store the plane's geometric characteristics and its convex hull.
 */
//template<typename Scalar>
class PlaneCHull : public Plane
{
  public:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ConvexHullPtr;
    std::vector<std::vector<size_t> > vv_hull_indices;
//    std::vector<cv::Point2D> vConvexHull2D;
};

struct TPlaneSegmentationParams
{
    //params for integral normal estimation
    int normal_estimation_method;
    bool depth_dependent_smoothing;
    double max_depth_change_factor;
    double normal_smoothing_size;

    //params for organized multiplane segmentation
    double angle_threshold;
    double dist_threshold;
    double min_inliers_frac;
    double max_curvature;
};

size_t segmentPlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud, const TPlaneSegmentationParams & params, std::vector<PlaneCHull> & planes);
