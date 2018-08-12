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

CCalibFromPlanesGui::CCalibFromPlanesGui(CObservationTreeGui *model, TCalibFromPlanesParams *params) :
    CCalibFromPlanes(model,  params)
{}

CCalibFromPlanesGui::~CCalibFromPlanesGui()
{}

void CCalibFromPlanesGui::addTextObserver(CTextObserver *observer)
{
	m_text_observers.push_back(observer);
}

void CCalibFromPlanesGui::addPlanesObserver(CPlanesObserver *observer)
{
	m_planes_observers.push_back(observer);
}

void CCalibFromPlanesGui::addCorrespPlanesObserver(CCorrespPlanesObserver *observer)
{
	m_corresp_planes_observers.push_back(observer);
}

void CCalibFromPlanesGui::publishText(const std::string &msg)
{
	for(CTextObserver *observer : m_text_observers)
	{
		observer->onReceivingText(msg);
	}
}

void CCalibFromPlanesGui::publishPlanes(const int &sensor_id, const int &sync_obs_id)
{
	for(CPlanesObserver *observer : m_planes_observers)
	{
		observer->onReceivingPlanes(sensor_id, mvv_planes[sensor_id][sync_obs_id]);
	}
}

void CCalibFromPlanesGui::publishCorrespPlanes(const int &obs_set_id)
{
	std::map<int,std::map<int,std::vector<std::array<CPlaneCHull,2>>>> corresp_planes;

	for(std::map<int,std::map<int,std::vector<std::array<int,3>>>>::iterator it_sensor_i = mmv_plane_corresp.begin();
	    it_sensor_i != mmv_plane_corresp.end(); it_sensor_i++)
	{
		int sensor_i = it_sensor_i->first;
		for(std::map<int,std::vector<std::array<int,3>>>::iterator it_sensor_j = it_sensor_i->second.begin();
		    it_sensor_j != it_sensor_i->second.end(); it_sensor_j++)
		{
			int sensor_j = it_sensor_j->first;
			std::vector<std::array<int,3>> &correspondences = it_sensor_j->second;
			for(int i = 0; i < correspondences.size(); i++)
			{
				int set_id = correspondences[i][0];
				if(set_id == obs_set_id)
				{
					int sync_obs1_id = sync_model->findSyncIndexFromSet(set_id, sync_model->getSensorLabels()[sensor_i]);
					int sync_obs2_id = sync_model->findSyncIndexFromSet(set_id, sync_model->getSensorLabels()[sensor_j]);

					std::array<CPlaneCHull,2> planes_pair{mvv_planes[sensor_i][sync_obs1_id][correspondences[i][1]],
						                                 mvv_planes[sensor_j][sync_obs2_id][correspondences[i][2]]};
					corresp_planes[sensor_i][sensor_j].push_back(planes_pair);
				}
			}
		}
	}

	for(CCorrespPlanesObserver *observer : m_corresp_planes_observers)
	{
		observer->onReceivingCorrespPlanes(corresp_planes, sync_model->getSensorPoses());
	}
}

CalibFromPlanesStatus CCalibFromPlanesGui::calibStatus()
{
	return params->calib_status;
}

void CCalibFromPlanesGui::run()
{
	// For running all steps at a time
}

void CCalibFromPlanesGui::extractPlanes()
{
	publishText("****Running plane segmentation algorithm****");

	CObservationTreeItem *root_item, *tree_item, *item;
	CObservation3DRangeScan::Ptr obs_item;
	size_t sync_obs_id = 0;
	root_item = sync_model->getRootItem();

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

	selected_sensor_labels = sync_model->getSensorLabels();
	std::vector<int> used_sets;

	for(size_t i = 0; i < selected_sensor_labels.size(); i++)
	{
		publishText("**Extracting planes from " + selected_sensor_labels[i] + " observations**");
		mvv_planes[i].resize((sync_model->getSyncIndices()[i]).size());

		for(size_t j = 0; j < 15; j += params->downsample_factor)
		{
			tree_item = root_item->child(j);

			for(size_t k = 0; k < tree_item->childCount(); k++)
			{
				item = tree_item->child(k);
				obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(item->getObservation());
				if((obs_item->sensorLabel == selected_sensor_labels[i]) && (obs_item->timestamp != prev_ts))
				{
					//if(item->cloud() != nullptr)
					   // cloud  = item->cloud();

					//else
					//{
						obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);
						cloud->is_dense = false;
						//item->setCloud(cloud);
					//}

					plane_segment_start = pcl::getTime();
					segmented_planes.clear();
					segmentPlanes(cloud, segmented_planes);
					plane_segment_end = pcl::getTime();

					n_planes = segmented_planes.size();
					publishText(std::to_string(n_planes) + " plane(s) extracted from observation #" + std::to_string(tree_item->child(k)->getPriorIndex())
					            + "\nTime elapsed: " +  std::to_string(plane_segment_end - plane_segment_start));

					sync_obs_id = sync_model->findSyncIndexFromSet(j, obs_item->sensorLabel);
					mvv_planes[i][sync_obs_id] = segmented_planes;
					prev_ts = obs_item->timestamp;
				}
			}

			if(i == 0)
				used_sets.push_back(j);
		}

		sync_obs_id = 0;
	}

	std::string s = "Used sets:\n";
	for(auto iter = used_sets.begin(); iter != used_sets.end(); iter++)
		s = s + std::to_string((*iter)) + " ";

	publishText(s);

	params->calib_status = CalibFromPlanesStatus::PLANES_EXTRACTED;
}

void CCalibFromPlanesGui::matchPlanes()
{
	publishText("****Running plane matching algorithm****");

	std::vector<Eigen::Matrix4f> sensor_poses = sync_model->getSensorPoses();
	std::vector<std::string> sensor_labels;

	CObservationTreeItem *root_item, *tree_item, *item;
	int sync_obs_id, sensor_id;
	root_item = sync_model->getRootItem();
	sensor_labels = sync_model->getSensorLabels();

	std::vector<std::vector<CPlaneCHull>> planes;
	std::vector<int> used_sets;

	//for(int i = 0; i < root_item->childCount(); i++)
	for(int i = 0; i < 15; i+= params->downsample_factor)
	{
		tree_item = root_item->child(i);
		planes.resize(sensor_labels.size());

		publishText("**Finding matches between planes in set #" + std::to_string(i) + "**");

		for(int j = 0; j < tree_item->childCount(); j++)
		{
			item = tree_item->child(j);
			sensor_id = utils::findItemIndexIn(sensor_labels, item->getObservation()->sensorLabel);
			sync_obs_id = sync_model->findSyncIndexFromSet(i, item->getObservation()->sensorLabel);
			planes[sensor_id] = mvv_planes[sensor_id][sync_obs_id];
		}

		findPotentialMatches(planes, i);
		planes.clear();

		//print statistics
		int count = 0;
		for(std::map<int,std::map<int,std::vector<std::array<int,3>>>>::iterator iter1 = mmv_plane_corresp.begin(); iter1 != mmv_plane_corresp.end(); iter1++)
		{
			for(std::map<int,std::vector<std::array<int,3>>>::iterator iter2 = iter1->second.begin(); iter2 != iter1->second.end(); iter2++)
			{
				for(int k = 0; k < iter2->second.size(); k++)
				{
					if(iter2->second[k][0] == i)
						count++;
				}

				publishText(std::to_string(count) + " matches found between " + sensor_labels[iter1->first] + " and " + sensor_labels[iter2->first]);
				count = 0;
			}
		}

		used_sets.push_back(i);
	}

	std::string s = "Used sets:\n";
	for(auto iter = used_sets.begin(); iter != used_sets.end(); iter++)
		s = s + std::to_string((*iter)) + " ";

	publishText(s);

	params->calib_status = CalibFromPlanesStatus::PLANES_MATCHED;
}

void CCalibFromPlanesGui::calibrate()
{
	publishText("****Running the calibration solver****");
	computeRotation();

	publishText("**Results of the rotation solver**");

	std::string stats;
	std::stringstream stream;

	for(int sensor_id = 0; sensor_id < sync_model->getNumberOfSensors(); sensor_id++)
		stream << result.estimate[sensor_id].block(0,0,3,3) << "\n";

	stats = "Status: " + result.msg;
	stats += "\nInitial error: " + std::to_string(result.init_error);
	stats += "\nNumber of iterations: " + std::to_string(result.num_iters);
	stats += "\nFinal error: " + std::to_string(result.final_error);
	stats += "\n\nEstimated rotation: \n";
	stats += stream.str();

	publishText(stats);
}
