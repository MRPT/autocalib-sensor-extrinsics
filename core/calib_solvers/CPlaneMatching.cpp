#include <calib_solvers/CPlaneMatching.h>

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/math/types_math.h>
#include <mrpt/maps/PCL_adapters.h>
#include <mrpt/pbmap/PbMap.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/time.h>

using namespace mrpt::obs;

CPlaneMatching::CPlaneMatching(CObservationTreeModel *model, std::array<double, 6> init_calib, TPlaneMatchingParams params)
{
	m_model = model;
	m_init_calib = init_calib;
	m_params = params;
}

CPlaneMatching::~CPlaneMatching()
{
}

void CPlaneMatching::addTextObserver(CTextObserver *observer)
{
	m_text_observers.push_back(observer);
}

void CPlaneMatching::addPlanesObserver(CPlanesObserver *observer)
{
	m_planes_observers.push_back(observer);
}

void CPlaneMatching::publishText(const std::string &msg)
{
	for(CTextObserver *observer : m_text_observers)
	{
		observer->onReceivingText(msg);
	}
}

void CPlaneMatching::publishPlaneCloud(const int &set_num, const int &cloud_num, const int &sensor_id)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
	cloud = m_extracted_plane_clouds_sets[set_num][cloud_num];

	for(CPlanesObserver *observer : m_planes_observers)
	{
		observer->onReceivingPlaneCloud(sensor_id, cloud);
	}
}

void CPlaneMatching::run()
{
	// For running all steps at a time
}

void CPlaneMatching::extractPlanes()
{
	publishText("****Running plane matching calibration algorithm****");

	CObservationTreeItem *root_item, *tree_item;
	CObservation3DRangeScan::Ptr obs_item;
	root_item = m_model->getRootItem();

	T3DPointsProjectionParams params;
	params.MAKE_DENSE = false;
	params.MAKE_ORGANIZED = true;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> extracted_plane_clouds;

	//for(int i = 0; i < root_item->childCount(); i++)
	// using only few observations for memory reasons
	for(int i = 0; i < 17; i++)
	{
		tree_item = root_item->child(i);
		extracted_plane_clouds.clear();

		for(int j = 0; j < tree_item->childCount(); j++)
		{
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr extracted_planes(new pcl::PointCloud<pcl::PointXYZRGBA>);
			publishText("**Extracting planes from #" + std::to_string(j) + " observation in observation set #" + std::to_string(i) + "**");

			obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(tree_item->child(j)->getObservation());
			obs_item->project3DPointsFromDepthImageInto(*cloud, params);
			runSegmentation(cloud, extracted_planes);
			extracted_plane_clouds.push_back(extracted_planes);
		}

		m_extracted_plane_clouds_sets.push_back(extracted_plane_clouds);
	}
}

void CPlaneMatching::runSegmentation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &extracted_planes)
{
	double plane_extract_start = pcl::getTime();
	unsigned min_inliers = m_params.min_inliers_frac * cloud->size();

	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimation;

	if(m_params.normal_estimation_method == 0)
		normal_estimation.setNormalEstimationMethod(normal_estimation.COVARIANCE_MATRIX);
	else if(m_params.normal_estimation_method == 1)
		normal_estimation.setNormalEstimationMethod(normal_estimation.AVERAGE_3D_GRADIENT);
	else
		normal_estimation.setNormalEstimationMethod(normal_estimation.AVERAGE_DEPTH_CHANGE);

	normal_estimation.setDepthDependentSmoothing(m_params.depth_dependent_smoothing);
	normal_estimation.setMaxDepthChangeFactor(m_params.max_depth_change_factor);
	normal_estimation.setNormalSmoothingSize(m_params.normal_smoothing_size);

	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
	normal_estimation.setInputCloud(cloud);
	normal_estimation.compute(*normal_cloud);

	pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> multi_plane_segmentation;
	multi_plane_segmentation.setMinInliers(min_inliers);
	multi_plane_segmentation.setAngularThreshold(m_params.angle_threshold);
	multi_plane_segmentation.setDistanceThreshold(m_params.dist_threshold);
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

    // Create a vector with the planes detected in this keyframe, and calculate their parameters (normal, center, pointclouds, etc.)

    mrpt::pbmap::PbMap pbmap; // Save all the segmented planes into a PbMap
    float max_curvature_plane = 0.03;
    for (size_t i = 0; i < regions.size (); i++)
    {
//      std::cout << "curv " << regions[i].getCurvature() << std::endl;
      if(regions[i].getCurvature() > max_curvature_plane)
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
//    cout << "Extract inliers\n";

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
      contourPtr->points = regions[i].getContour();
      plane.calcConvexHull(contourPtr);
      plane.computeMassCenterAndArea();

      // Check whether this region correspond to the same plane as a previous one (this situation may happen when there exists a small discontinuity in the observation)
      bool isSamePlane = false;
      for (size_t j = 0; j < pbmap.vPlanes.size(); j++)
        if( pbmap.vPlanes[j].isSamePlane(plane, 0.998, 0.1, 0.4) ) // The planes are merged if they are the same
        {
//          cout << "Merge local region\n";
          isSamePlane = true;
          pbmap.vPlanes[j].mergePlane(plane);

          break;
        }
      if(!isSamePlane)
      {
//          plane.calcMainColor();
        plane.id = pbmap.vPlanes.size();
        pbmap.vPlanes.push_back(plane);
      }
    }

	std::stringstream stream;
	stream << inlier_indices.size() << " plane(s) detected\n" << "Time elapsed: " << double(plane_extract_end - plane_extract_start) << std::endl;

	publishText(stream.str());

	for(size_t i = 0; i < inlier_indices.size(); i++)
	{
		std::vector<int> indices = inlier_indices[i].indices;
		for(size_t j = 0; j < indices.size(); j++)
		{
			extracted_planes->points.push_back(cloud->points[indices[j]]);
		}
	}
}

void CPlaneMatching::proceed()
{
}
