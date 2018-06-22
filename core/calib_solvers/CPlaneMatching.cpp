#include <calib_solvers/CPlaneMatching.h>

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/math/types_math.h>
#include <mrpt/maps/PCL_adapters.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/time.h>

using namespace mrpt::obs;
using namespace mrpt::maps;

CPlaneMatching::CPlaneMatching(CObservationTreeModel *model, std::array<double, 6> init_calib, TPlaneMatchingParams params)
{
	m_model = model;
	m_init_calib = init_calib;
	m_params = params;
}

CPlaneMatching::~CPlaneMatching()
{
}

void CPlaneMatching::addSubscriber(CSubscriber *subscriber)
{
	m_subscribers.push_back(subscriber);
}

void CPlaneMatching::publishEvent(const std::string &msg)
{
	for(CSubscriber *subscriber : m_subscribers)
	{
		subscriber->onEvent(msg);
	}
}

void CPlaneMatching::run()
{

	publishEvent("**Running plane matching calibration algorithm**");
	CObservationTreeItem *root_item, *tree_item;
	CObservation3DRangeScan::Ptr obs_item;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	std::stringstream stream;
	T3DPointsProjectionParams projection_params;
	projection_params.MAKE_DENSE = false;
	projection_params.MAKE_ORGANIZED = true;

	root_item = m_model->getRootItem();

	//for(int i = 0; i < m_model->rowCount(); i++)
	//{
		//tree_item = root_item->child(i);
		//for(int j = 0; j < tree_item->childCount(); j++)
		//{
			//stream << "Extracting planes from #" << j << " observation in obesrvation set #" << i << "..";
			//sendTextUpdate(stream.str());

			//obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(tree_item->child(j)->getObservation());
			//obs_item->load();
			//obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);
			//obs_item->unload();

			//detectPlanes(cloud, stream);
		//}
	//}
	
	// Testing with just the first observation
	
	tree_item = root_item->child(0);
	stream << "Extracting planes from #0 observation in observation set #0..";
	publishEvent(stream.str());

	obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(tree_item->child(0)->getObservation());
	obs_item->load();
	obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);
	obs_item->unload();

	//detectPlanes(cloud);
}

void CPlaneMatching::detectPlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
//	double plane_extract_start = pcl::getTime();
//	unsigned min_inliers = m_params.min_inliers_frac * cloud->size();

//	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimation;
//	normal_estimation.setNormalEstimationMethod(normal_estimation.COVARIANCE_MATRIX);
//	normal_estimation.setMaxDepthChangeFactor(0.02f);
//	normal_estimation.setNormalSmoothingSize(10.0f);
//	normal_estimation.setDepthDependentSmoothing(true);

//	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
//	normal_estimation.setInputCloud(cloud);
//	normal_estimation.compute(*normal_cloud);

//	pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> multi_plane_segmentation;
//	multi_plane_segmentation.setMinInliers(min_inliers);
//	multi_plane_segmentation.setAngularThreshold(m_params.angle_threshold);
//	multi_plane_segmentation.setDistanceThreshold(m_params.dist_threshold);
//	multi_plane_segmentation.setInputNormals(normal_cloud);
//	multi_plane_segmentation.setInputCloud(cloud);

//	std::vector<pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA>>> regions;
//	std::vector<pcl::ModelCoefficients> model_coefficients;
//	std::vector<pcl::PointIndices> inlier_indices;
//	pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
//	std::vector<pcl::PointIndices> label_indices;
//	std::vector<pcl::PointIndices> boundary_indices;

//	multi_plane_segmentation.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
//	double plane_extract_end = pcl::getTime();

//	std::stringstream stream;
//	stream << regions.size() << " plane(s) detected\n" << "Time elapsed: " << double(plane_extract_end - plane_extract_start) << std::endl;

//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

//	segmented_cloud->points = cloud->points;

//	for(size_t i = 0; i < inlier_indices.size(); i++)
//	{
//		std::vector<int> indices = inlier_indices[i].indices;
//		for(size_t j = 0; j < indices.size(); j++)
//		{
//			(segmented_cloud->points[indices[j]]).r = 255;
//			(segmented_cloud->points[indices[j]]).g = 0;
//			(segmented_cloud->points[indices[j]]).b = 0;
//		}
//	}
}

void CPlaneMatching::proceed()
{
}
