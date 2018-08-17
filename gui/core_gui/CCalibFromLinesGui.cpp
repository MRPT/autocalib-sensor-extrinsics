#include <core_gui/CCalibFromLinesGui.h>

#include <mrpt/obs/CObservation3DRangeScan.h>

using namespace mrpt::obs;

CCalibFromLinesGui::CCalibFromLinesGui(CObservationTree *model, TCalibFromLinesParams *params) :
    CCalibFromLines(model, params)
{}

CCalibFromLinesGui::~CCalibFromLinesGui()
{}

void CCalibFromLinesGui::addTextObserver(CTextObserver *observer)
{
	m_text_observers.push_back(observer);
}

void CCalibFromLinesGui::addLinesObserver(CLinesObserver *observer)
{
	m_lines_observers.push_back(observer);
}

void CCalibFromLinesGui::addCorrespLinesObserver(CCorrespLinesObserver *observer)
{
	m_corresp_lines_observers.push_back(observer);
}

void CCalibFromLinesGui::publishText(const std::string &msg)
{
	for(CTextObserver *observer : m_text_observers)
	{
		observer->onReceivingText(msg);
	}
}

void CCalibFromLinesGui::publishLines(const int &sensor_id, const int &sync_obs_id)
{
	for(CLinesObserver *observer : m_lines_observers)
	{
		observer->onReceivingLines(sensor_id, mvv_lines[sensor_id][sync_obs_id]);
	}
}

void CCalibFromLinesGui::publishCorrespLines(const int &obs_set_id)
{
	std::map<int,std::map<int,std::vector<std::array<CLine,2>>>> corresp_lines;

	for(std::map<int,std::map<int,std::vector<std::array<int,3>>>>::iterator it_sensor_i = mmv_line_corresp.begin();
	    it_sensor_i != mmv_line_corresp.end(); it_sensor_i++)
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

					std::array<CLine,2> lines_pair{mvv_lines[sensor_i][sync_obs1_id][correspondences[i][1]],
						                                 mvv_lines[sensor_j][sync_obs2_id][correspondences[i][2]]};
					corresp_lines[sensor_i][sensor_j].push_back(lines_pair);
				}
			}
		}
	}

	for(CCorrespLinesObserver *observer : m_corresp_lines_observers)
	{
		observer->onReceivingCorrespLines(corresp_lines, sync_model->getSensorPoses());
	}
}

CalibFromLinesStatus CCalibFromLinesGui::calibStatus()
{
	return params->calib_status;
}

void CCalibFromLinesGui::extractLines()
{
	publishText("****Running line segmentation algorithm****");

	CObservationTreeItem *root_item, *tree_item, *item;
	CObservation3DRangeScan::Ptr obs_item;
	mrpt::math::CMatrixDouble44 intensity_to_depth_rt;
	Eigen::Affine3f intensity_to_depth_transform;
	size_t sync_obs_id = 0;
	root_item = sync_model->getRootItem();

	std::vector<CLine> segmented_lines;
	size_t n_lines;
	double line_segment_start, line_segment_end;

	std::vector<std::string> selected_sensor_labels;
	std::string prev_sensor_label;
	mrpt::system::TTimeStamp prev_ts = 0;

	selected_sensor_labels = sync_model->getSensorLabels();
	std::vector<int> used_sets;

	for(size_t i = 0; i < selected_sensor_labels.size(); i++)
	{
		publishText("**Extracting lines from " + selected_sensor_labels[i] + " observations**");
		mvv_lines[i].resize((sync_model->getSyncIndices()[i]).size());

		//let's run it for 15 sets
		for(size_t j = 0; j < 15; j += params->downsample_factor)
		{
			tree_item = root_item->child(j);

			for(size_t k = 0; k < tree_item->childCount(); k++)
			{
				item = tree_item->child(k);
				obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(item->getObservation());
				if((obs_item->sensorLabel == selected_sensor_labels[i]) && (obs_item->timestamp != prev_ts))
				{
					cv::Mat image = cv::cvarrToMat(obs_item->intensityImage.getAs<IplImage>());
					Eigen::MatrixXf range = obs_item->rangeImage;
					(obs_item->relativePoseIntensityWRTDepth).getHomogeneousMatrix(intensity_to_depth_rt);
					intensity_to_depth_transform = intensity_to_depth_rt.matrix().cast<float>();

					line_segment_start = pcl::getTime();
					segmented_lines.clear();
					segmentLines(image, range, obs_item->cameraParamsIntensity, intensity_to_depth_transform, segmented_lines);
					line_segment_end = pcl::getTime();

					n_lines = segmented_lines.size();
					publishText(std::to_string(n_lines) + " line(s) extracted from observation #" + std::to_string(tree_item->child(k)->getPriorIndex())
					            + "\nTime elapsed: " +  std::to_string(line_segment_end - line_segment_start));

					sync_obs_id = sync_model->findSyncIndexFromSet(j, obs_item->sensorLabel);
					mvv_lines[i][sync_obs_id] = segmented_lines;
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

	params->calib_status = CalibFromLinesStatus::LINES_EXTRACTED;
}

void CCalibFromLinesGui::matchLines()
{
	publishText("****Running line matching algorithm****");

	std::vector<Eigen::Matrix4f> sensor_poses = sync_model->getSensorPoses();
	std::vector<std::string> sensor_labels;

	CObservationTreeItem *root_item, *tree_item, *item;
	int sync_obs_id, sensor_id;
	root_item = sync_model->getRootItem();
	sensor_labels = sync_model->getSensorLabels();

	std::vector<std::vector<CLine>> lines;
	std::vector<int> used_sets;

	//for(int i = 0; i < root_item->childCount(); i++)
	for(int i = 0; i < 15; i += params->downsample_factor)
	{
		tree_item = root_item->child(i);
		lines.resize(sensor_labels.size());

		publishText("**Finding matches between lines in set #" + std::to_string(i) + "**");

		for(int j = 0; j < tree_item->childCount(); j++)
		{
			item = tree_item->child(j);
			sensor_id = utils::findItemIndexIn(sensor_labels, item->getObservation()->sensorLabel);
			sync_obs_id = sync_model->findSyncIndexFromSet(i, item->getObservation()->sensorLabel);
			lines[sensor_id] = mvv_lines[sensor_id][sync_obs_id];
		}

		findPotentialMatches(lines, i);
		lines.clear();

		//print statistics
		int count = 0;

		for(std::map<int,std::map<int,std::vector<std::array<int,3>>>>::iterator iter1 = mmv_line_corresp.begin(); iter1 != mmv_line_corresp.end(); iter1++)
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

	params->calib_status = CalibFromLinesStatus::LINES_MATCHED;
}
