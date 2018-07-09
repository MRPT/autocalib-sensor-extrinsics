/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "planes.h"
#include <mrpt/pbmap/PbMap.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/time.h>

//using namespace SegmentPlanes;
size_t segmentPlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const TPlaneSegmentationParams & params, std::vector<PlaneCHull> & planes)
{
    double plane_extract_start = pcl::getTime();
    unsigned min_inliers = params.min_inliers_frac * cloud->size();

    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimation;

    if(params.normal_estimation_method == 0)
        normal_estimation.setNormalEstimationMethod(normal_estimation.COVARIANCE_MATRIX);
    else if(params.normal_estimation_method == 1)
        normal_estimation.setNormalEstimationMethod(normal_estimation.AVERAGE_3D_GRADIENT);
    else
        normal_estimation.setNormalEstimationMethod(normal_estimation.AVERAGE_DEPTH_CHANGE);

    normal_estimation.setDepthDependentSmoothing(params.depth_dependent_smoothing);
    normal_estimation.setMaxDepthChangeFactor(params.max_depth_change_factor);
    normal_estimation.setNormalSmoothingSize(params.normal_smoothing_size);

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    normal_estimation.setInputCloud(cloud);
    normal_estimation.compute(*normal_cloud);

    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> multi_plane_segmentation;
    multi_plane_segmentation.setMinInliers(min_inliers);
    multi_plane_segmentation.setAngularThreshold(params.angle_threshold);
    multi_plane_segmentation.setDistanceThreshold(params.dist_threshold);
    multi_plane_segmentation.setInputNormals(normal_cloud);
    multi_plane_segmentation.setInputCloud(cloud);

    std::vector<pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA>>> regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;

    multi_plane_segmentation.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
    double plane_extract_end = pcl::getTime();

    // Create a vector with the planes detected in this frame, and calculate their parameters (normal, center, pointclouds, etc.)
    //std::cout << regions.size() << " planes \n";
    mrpt::pbmap::PbMap pbmap; // Save all the segmented planes into a PbMap
    planes.resize( regions.size() );
    for (size_t i = 0; i < regions.size(); i++)
    {
        PlaneCHull planehull;

        //      std::cout << "curv " << regions[i].getCurvature() << std::endl;
        if(regions[i].getCurvature() > params.max_curvature)
            continue;

        mrpt::pbmap::Plane plane;
        plane.v3center = regions[i].getCentroid ();
        plane.v3normal = Eigen::Vector3f(model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2]);
        plane.d = model_coefficients[i].values[3];
        // Force the normal vector to point towards the camera
        if( model_coefficients[i].values[3] < 0)
        {
            plane.v3normal = -plane.v3normal;
            plane.d = -plane.d;
        }
        plane.curvature = regions[i].getCurvature();
        //      cout << "normal " << plane.v3normal.transpose() << " center " << regions[i].getCentroid().transpose() << " " << plane.v3center.transpose() << endl;
        //    cout << "D " << -(plane.v3normal.dot(plane.v3center)) << " " << plane.d << endl;

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        extract.setInputCloud ( cloud );
        extract.setIndices ( boost::make_shared<const pcl::PointIndices> (inlier_indices[i]) );
        extract.setNegative (false);
        extract.filter (*plane.planePointCloudPtr);    // Write the planar point cloud
        plane.inliers = inlier_indices[i].indices;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
        contourPtr->points = regions[i].getContour();
        std::vector<size_t> indices_convex_hull;
        plane.calcConvexHull(contourPtr, indices_convex_hull);
        plane.computeMassCenterAndArea();
        //std::cout << "Extract inliers " << inlier_indices[i].indices.size() << " " << boundary_indices[i].indices.size() << " " << contourPtr->points.size() << "\n";

        for (size_t j = 0; j < indices_convex_hull.size(); j++)
            indices_convex_hull[j] = boundary_indices[i].indices[indices_convex_hull[j]];

        //std::cout << i << " normal " << plane.v3normal.transpose() << "\n";
        planes[i].v3normal = plane.v3normal;
        planes[i].d = plane.d;
        planes[i].vv_hull_indices.push_back(indices_convex_hull);

//        // Check whether this region correspond to the same plane as a previous one (this situation may happen when there exists a small discontinuity in the observation)
//        bool isSamePlane = false;
//        for (size_t j = 0; j < pbmap.vPlanes.size(); j++)
//            if( pbmap.vPlanes[j].isSamePlane(plane, 0.998, 0.1, 0.4) ) // The planes are merged if they are the same
//            {
//                std::cout << "Merge local region\n\n";
//                isSamePlane = true;
//                pbmap.vPlanes[j].mergePlane(plane);
//                break;
//            }
//        if(!isSamePlane)
//            pbmap.vPlanes.push_back(plane);
    }

//    planes.resize( pbmap.vPlanes.size() );
//    for (size_t i = 0; i < pbmap.vPlanes.size (); i++)
//    {
//        std::cout << i << " normal " << pbmap.vPlanes[i].v3normal.transpose() << "\n";
//        planes[i].v3normal = pbmap.vPlanes[i].v3normal;
//        planes[i].d = pbmap.vPlanes[i].d;
//        planes[i].vv_hull_indices.push_back(indices_convex_hull);
//    }

    std::stringstream stream;
    stream << inlier_indices.size() << " plane(s) detected\n" << "Time elapsed: " << double(plane_extract_end - plane_extract_start) << std::endl;

    //publishText(stream.str());

//    for(size_t i = 0; i < inlier_indices.size(); i++)
//    {
//        std::vector<int> indices = inlier_indices[i].indices;
//        for(size_t j = 0; j < indices.size(); j++)
//        {
//            extracted_planes->points.push_back(cloud->points[indices[j]]);
//        }
//    }

    return planes.size();
}
