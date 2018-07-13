#include "CCalibFromPlanesGui.h"

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

#include <thread>

using namespace mrpt::obs;

CCalibFromPlanesGui::CCalibFromPlanesGui(CObservationTreeModel *model, TPlaneMatchingParams params) :
    CCalibFromPlanes(2)
{
	m_model = model;
	m_params = params;
}

CCalibFromPlanesGui::~CCalibFromPlanesGui()
{
}

void CCalibFromPlanesGui::addTextObserver(CTextObserver *observer)
{
	m_text_observers.push_back(observer);
}

void CCalibFromPlanesGui::addPlanesObserver(CPlanesObserver *observer)
{
	m_planes_observers.push_back(observer);
}

void CCalibFromPlanesGui::publishText(const std::string &msg)
{
	for(CTextObserver *observer : m_text_observers)
	{
		observer->onReceivingText(msg);
	}
}

void CCalibFromPlanesGui::publishPlaneCloud(const int &set_num, const int &cloud_num, const int &sensor_id)
{
	CObservationTreeItem *root_item;
	root_item = m_model->getRootItem();

	CObservation3DRangeScan::Ptr obs_item;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obs_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	T3DPointsProjectionParams projection_params;
	projection_params.MAKE_DENSE = false;
	projection_params.MAKE_ORGANIZED = false;

	obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(root_item->child(set_num)->child(cloud_num)->getObservation());
	obs_item->project3DPointsFromDepthImageInto(*obs_cloud, projection_params);
	obs_cloud->is_dense = false;


	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> plane_cloud;

	for(size_t i = 0; i < vvv_planes[set_num][cloud_num].size(); i++)
	{
		std::vector<int> &indices = vvv_planes[set_num][cloud_num][i].v_inliers;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBA>);

		for(size_t k = 0; k < indices.size(); k++)
		{
			plane->points.push_back(obs_cloud->points[indices[k]]);
		}

		plane_cloud.push_back(plane);
	}

	for(CPlanesObserver *observer : m_planes_observers)
	{
		observer->onReceivingPlaneCloud(sensor_id, plane_cloud);
	}
}

void CCalibFromPlanesGui::run()
{
	// For running all steps at a time
}

void CCalibFromPlanesGui::extractPlanes()
{
	//std::thread thr0(&CCalibFromPlanesGui::publishText, this, "****Running plane segmentation algorithm****");
	//thr0.join();

	publishText("****Running plane segmentation algorithm****");

	CObservationTreeItem *root_item, *tree_item;
	CObservation3DRangeScan::Ptr obs_item;
	root_item = m_model->getRootItem();

	T3DPointsProjectionParams projection_params;
	projection_params.MAKE_DENSE = false;
	projection_params.MAKE_ORGANIZED = true;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    TPlaneSegmentationParams seg_params;
    seg_params.normal_estimation_method = m_params.normal_estimation_method;
    seg_params.depth_dependent_smoothing = m_params.depth_dependent_smoothing;
    seg_params.max_depth_change_factor = m_params.max_depth_change_factor;
    seg_params.normal_smoothing_size = m_params.normal_smoothing_size;
    seg_params.angle_threshold = m_params.angle_threshold;
    seg_params.dist_threshold = m_params.dist_threshold;
    seg_params.min_inliers_frac = m_params.min_inliers_frac;
    seg_params.max_curvature = 0.1;

	size_t n_planes;

	vvv_planes.resize(15);

	// using only few observations for memory reasons
	for(size_t i = 0; i < 15; i++)
	{
		tree_item = root_item->child(i);

		vvv_planes[i] = std::vector< std::vector< CPlaneCHull > >(tree_item->childCount());

		for(size_t j = 0; j < tree_item->childCount(); j++)
		{
			publishText("**Extracting planes from #" + std::to_string(j) + " observation in observation set #" + std::to_string(i) + "**");
			//std::thread thr(&CCalibFromPlanesGui::publishText, this, "**Extracting planes from #" + std::to_string(j) + " observation in observation set #" + std::to_string(i) + "**");
			//thr.join();

			obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(tree_item->child(j)->getObservation());
			obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);

			n_planes = segmentPlanes(cloud, seg_params, vvv_planes[i][j]);
			publishText(std::to_string(n_planes) + " plane(s) extracted");

			//std::thread thr2(&CCalibFromPlanesGui::publishText, this, std::to_string(n_planes) + " plane(s) extracted");
			//thr2.join();
        }
	}
}

void CCalibFromPlanesGui::proceed()
{
	//calib.findPotentialMatches(calib.vvv_planes[i], i);
	//calib.computeCalibration_rot(ExtrinsicCalib::m_init_calib);
}
