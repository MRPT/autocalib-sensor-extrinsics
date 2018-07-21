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

CCalibFromPlanesGui::CCalibFromPlanesGui(CObservationTreeGui *model, const TCalibFromPlanesParams &params) :
    CCalibFromPlanes(model->getSensorLabels().size())
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

void CCalibFromPlanesGui::publishPlanes(const int &sensor_id, const int &obs_id)
{
	CObservationTreeItem *root_item;
	root_item = m_sync_model->getRootItem();

	int sync_obs_id = cutils::findItemIndexIn(m_sync_model->getSyncIndices()[sensor_id], obs_id);

	std::vector<CPlaneCHull> planes;
	planes = vvv_planes[sensor_id][sync_obs_id];

	for(CPlanesObserver *observer : m_planes_observers)
	{
		observer->onReceivingPlanes(sensor_id, planes);
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
	size_t sync_obs_id = 0;
	root_item = m_sync_model->getRootItem();

	T3DPointsProjectionParams projection_params;
	projection_params.MAKE_DENSE = false;
	projection_params.MAKE_ORGANIZED = true;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	std::vector<CPlaneCHull> segmented_planes;
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
		vvv_planes[i].resize((m_sync_model->getSyncIndices()[i]).size());
		vv_clouds[i].resize((m_sync_model->getSyncIndices()[i]).size());

		//let's run it for 5 sets
		for(size_t j = 0; j < 5; j++)
		{
			tree_item = root_item->child(j);

			for(size_t k = 0; k < tree_item->childCount(); k++)
			{
				obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(tree_item->child(k)->getObservation());
				if((obs_item->sensorLabel == selected_sensor_labels[i]) && (obs_item->timestamp != prev_ts))
				{
					obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);

					plane_segment_start = pcl::getTime();
					segmentPlanes(cloud, m_params.seg, segmented_planes);
					plane_segment_end = pcl::getTime();

					n_planes = segmented_planes.size();
					publishText(std::to_string(n_planes) + " plane(s) extracted from observation #" + std::to_string(tree_item->child(k)->getPriorIndex())
					            + "\nTime elapsed: " +  std::to_string(plane_segment_end - plane_segment_start));

					vvv_planes[i][sync_obs_id] = segmented_planes;
					vv_clouds[i][sync_obs_id] = cloud;
					sync_obs_id++;
					prev_ts = obs_item->timestamp;
				}
			}
		}

		sync_obs_id = 0;
	}
}

void CCalibFromPlanesGui::matchPlanes()
{
}
