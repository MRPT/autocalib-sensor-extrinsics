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

CCalibFromPlanesGui::CCalibFromPlanesGui(CObservationTreeGui *model, TPlaneMatchingParams params) :
    CCalibFromPlanes(2)
{
	m_sync_model = model;
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
	root_item = m_sync_model->getRootItem();

	CObservation3DRangeScan::Ptr obs_item;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obs_cloud;

	obs_id = findItemIndexIn(m_sync_model->getSyncIndices()[sensor_id], obs_id);

	//for debugging
	publishText(std::to_string(sensor_id) + " " + std::to_string(obs_id));

	obs_cloud = vv_clouds[sensor_id][obs_id];

	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> plane_cloud;

	for(size_t i = 0; i < vvv_planes[sensor_id][obs_id].size(); i++)
	{
		std::vector<int> indices = vvv_planes[sensor_id][obs_id][i].v_inliers;
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
	root_item = m_sync_model->getRootItem();

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

	std::vector<CPlaneCHull> segmented_planes;
	std::vector<std::vector<CPlaneCHull>> sensor_segmented_planes;
	size_t n_planes;	
	double plane_segment_start, plane_segment_end;

	std::vector<std::string> selected_sensor_labels;
	std::string prev_sensor_label;
	mrpt::system::TTimeStamp prev_ts = 0;

	selected_sensor_labels = m_sync_model->getSensorLabels();
	vv_clouds.resize(selected_sensor_labels.size());
	vvv_planes.resize(selected_sensor_labels.size());

	for(size_t i = 0; i < selected_sensor_labels.size(); i++)
	{
		publishText("**Extracting planes from sensor #" + std::to_string(i) + " observations**");

		//let's run it for 5 sets
		for(size_t j = 0; j < 5; j++)
		{
			tree_item = root_item->child(j);

			for(size_t j = 0; j < tree_item->childCount(); j++)
			{
				obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(tree_item->child(j)->getObservation());
				if((obs_item->sensorLabel == selected_sensor_labels[i]) && (obs_item->timestamp != prev_ts))
				{
					obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);

					plane_segment_start = pcl::getTime();
					segmentPlanes(cloud, seg_params, segmented_planes);
					plane_segment_end = pcl::getTime();
					n_planes = segmented_planes.size();
					publishText(std::to_string(n_planes) + " plane(s) extracted from observation #" + std::to_string(tree_item->child(j)->getPriorIndex())
					            + "\nTime elapsed: " +  std::to_string(plane_segment_end - plane_segment_start));

					sensor_segmented_planes.push_back(segmented_planes);
					prev_ts = obs_item->timestamp;
					vv_clouds[i].push_back(cloud);
				}
			}
		}

		vvv_planes[i].push_back(segmented_planes);
		sensor_segmented_planes.clear();
	}
}

void CCalibFromPlanesGui::proceed()
{
	//calib.findPotentialMatches(calib.vvv_planes[i], i);
	//calib.computeCalibration_rot(ExtrinsicCalib::m_init_calib);
}
