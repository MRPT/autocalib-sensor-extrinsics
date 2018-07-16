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
#include <functional>

using namespace mrpt::obs;

CCalibFromPlanesGui::CCalibFromPlanesGui(CObservationTreeModel *model, std::vector<std::vector<int>> &sync_obs_indices, TPlaneMatchingParams params) :
    CCalibFromPlanes(2)
{
	m_model = model;
	m_sync_obs_indices = sync_obs_indices;
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

void CCalibFromPlanesGui::publishPlaneCloud(const int &sensor_id, int obs_id)
{
	CObservationTreeItem *root_item;
	root_item = m_model->getRootItem();

	CObservation3DRangeScan::Ptr obs_item;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obs_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	T3DPointsProjectionParams projection_params;
	projection_params.MAKE_DENSE = false;
	projection_params.MAKE_ORGANIZED = false;

	obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(root_item->child(obs_id)->getObservation());
	obs_item->project3DPointsFromDepthImageInto(*obs_cloud, projection_params);
	obs_cloud->is_dense = false;

	// for finding the equivalent index in vvv_planes[sensor_id]
	auto iter = std::find(m_sync_obs_indices[sensor_id].begin(), m_sync_obs_indices[sensor_id].end(), obs_id);
	obs_id = std::distance(m_sync_obs_indices[sensor_id].begin(), iter);

	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> plane_cloud;

	for(size_t i = 0; i < vvv_planes[sensor_id][obs_id].size(); i++)
	{
		std::vector<int> &indices = vvv_planes[sensor_id][obs_id][i].v_inliers;
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
	double plane_segment_start, plane_segment_end;

	vvv_planes.resize(m_sync_obs_indices.size());

	for(size_t i = 0; i < m_sync_obs_indices.size(); i++)
	{
		//vvv_planes[i] = std::vector< std::vector< CPlaneCHull > >(m_sync_obs_indices[i].size());
		//for(size_t j = 0; j < m_sync_obs_indices[i].size(); j++)


		publishText("**Extracting planes from sensor #" + std::to_string(i) + " observations**");
		vvv_planes[i] = std::vector< std::vector< CPlaneCHull > >(5);

		//runnning only for a few observations due to memory reasons
		for(size_t j = 0; j < 5; j++)
		{	
			obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(root_item->child(m_sync_obs_indices[i][j])->getObservation());
			obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);

			plane_segment_start = pcl::getTime();
			segmentPlanes(cloud, seg_params, vvv_planes[i][j]);
			plane_segment_end = pcl::getTime();
			n_planes = vvv_planes[i][j].size();
			publishText(std::to_string(n_planes) + " plane(s) extracted from observation #" + std::to_string(m_sync_obs_indices[i][j])
			            + "\nTime elapsed: " +  std::to_string(plane_segment_end - plane_segment_start));
        }
	}
}

void CCalibFromPlanesGui::proceed()
{
	//calib.findPotentialMatches(calib.vvv_planes[i], i);
	//calib.computeCalibration_rot(ExtrinsicCalib::m_init_calib);
}
